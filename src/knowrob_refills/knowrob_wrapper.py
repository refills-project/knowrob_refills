import os
import traceback
import yaml
from collections import OrderedDict
from multiprocessing import Lock

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
from tf2_geometry_msgs import do_transform_pose

from refills_perception_interface.not_hacks import add_separator_between_barcodes, add_edge_separators, \
    merge_close_separators, merge_close_shelf_layers
from refills_perception_interface.tfwrapper import transform_pose, lookup_pose, lookup_transform
from refills_perception_interface.utils import print_with_prefix, ordered_load
from rosprolog_client import Prolog, PrologException
from rospy_message_converter.message_converter import convert_dictionary_to_ros_message

MAP = 'map'
SHOP = 'shop'
SHELF_FLOOR = '{}:\'ShelfLayer\''.format(SHOP)
DM_MARKET = 'dmshop'
SHELF_BOTTOM_LAYER = '{}:\'DMShelfBFloor\''.format(DM_MARKET)
SHELF_SYSTEM = '{}:\'DMShelfFrame\''.format(DM_MARKET)
SHELFH200 = '{}:\'DMShelfH200\''.format(DM_MARKET)
SHELF_T5 = '{}:\'DMShelfT5\''.format(DM_MARKET)
SHELF_T6 = '{}:\'DMShelfT6\''.format(DM_MARKET)
SHELF_T7 = '{}:\'DMShelfT7\''.format(DM_MARKET)
SHELF_W60 = '{}:\'DMShelfW60\''.format(DM_MARKET)
SHELF_W75 = '{}:\'DMShelfW75\''.format(DM_MARKET)
SHELF_W100 = '{}:\'DMShelfW100\''.format(DM_MARKET)
SHELF_W120 = '{}:\'DMShelfW120\''.format(DM_MARKET)
SHELF_H = '{}:\'DMShelfH\''.format(DM_MARKET)
SHELF_L = '{}:\'DMShelfL\''.format(DM_MARKET)

SEPARATOR = '{}:\'DMShelfSeparator4Tiles\''.format(DM_MARKET)
MOUNTING_BAR = '{}:\'DMShelfMountingBar\''.format(DM_MARKET)
BARCODE = '{}:\'DMShelfLabel\''.format(DM_MARKET)
PERCEPTION_AFFORDANCE = '{}:\'DMShelfPerceptionAffordance\''.format(DM_MARKET)

OBJECT_ACTED_ON = '\'http://knowrob.org/kb/knowrob.owl#objectActedOn\''
GOAL_LOCATION = '\'http://knowrob.org/kb/knowrob.owl#goalLocation\''
DETECTED_OBJECT = '\'http://knowrob.org/kb/knowrob.owl#detectedObject\''

MAX_SHELF_HEIGHT = 1.35


class KnowRob(object):
    prefix = 'knowrob_wrapper'

    def __init__(self, initial_mongo_db=None, clear_roslog=True):
        super(KnowRob, self).__init__()
        if clear_roslog or initial_mongo_db is not None:
            # if '/rosprolog/query' in rosservice.get_service_list():
            #     self.print_with_prefix('kill knowrob so mongo can be initialized!')
            # while '/rosprolog/query' in rosservice.get_service_list():
            #     rospy.sleep(0.5)
            if clear_roslog:
                self.mongo_drop_database('roslog')
        if initial_mongo_db is not None:
            self.mongo_load_database(initial_mongo_db)
            self.print_with_prefix('restored mongo, start knowrob')
        self.separators = {}
        self.product_to_gtin = None
        self.gtin_to_product = None
        self.perceived_frame_id_map = {}
        self.print_with_prefix('waiting for knowrob')
        self.prolog = Prolog()
        self.print_with_prefix('knowrob showed up')
        rospy.sleep(5)
        self.query_lock = Lock()
        # rospy.wait_for_message('/visualization_marker_array')
        # self.reset_object_state_publisher = rospy.ServiceProxy('/visualization_marker_array',
        #                                                        Trigger)
        self.shelf_layer_from_facing = {}
        self.shelf_system_from_layer = {}
        self.init_product_gtin_map()
        if initial_mongo_db is not None:
            self.republish_tf()
        self.order_dict = None
        self.parse_shelf_order_yaml()

    def print_with_prefix(self, msg):
        """
        :type msg: str
        """
        print_with_prefix(msg, self.prefix)

    def once(self, q):
        r = self.all_solutions(q)
        if len(r) == 0:
            return []
        return r[0]

    def all_solutions(self, q):
        self.print_with_prefix(q)
        r = self.prolog.all_solutions(q)
        self.print_with_prefix('result: {}'.format(r))
        return r

    def pose_to_prolog(self, pose_stamped):
        """
        :type pose_stamped: PoseStamped
        :return: PoseStamped in a form the knowrob likes
        :rtype: str
        """
        if isinstance(pose_stamped, PoseStamped):
            return '[\'{}\',[{},{},{}], [{},{},{},{}]]'.format(pose_stamped.header.frame_id,
                                                               pose_stamped.pose.position.x,
                                                               pose_stamped.pose.position.y,
                                                               pose_stamped.pose.position.z,
                                                               pose_stamped.pose.orientation.x,
                                                               pose_stamped.pose.orientation.y,
                                                               pose_stamped.pose.orientation.z,
                                                               pose_stamped.pose.orientation.w)
        elif isinstance(pose_stamped, TransformStamped):
            return '[\'{}\', [{},{},{}], [{},{},{},{}]]'.format(pose_stamped.header.frame_id,
                                                                pose_stamped.transform.translation.x,
                                                                pose_stamped.transform.translation.y,
                                                                pose_stamped.transform.translation.z,
                                                                pose_stamped.transform.rotation.x,
                                                                pose_stamped.transform.rotation.y,
                                                                pose_stamped.transform.rotation.z,
                                                                pose_stamped.transform.rotation.w)

    def prolog_to_pose_msg(self, query_result):
        """
        :type query_result: list
        :rtype: PoseStamped
        """
        ros_pose = PoseStamped()
        ros_pose.header.frame_id = query_result[0]
        ros_pose.pose.position = Point(*query_result[1])
        ros_pose.pose.orientation = Quaternion(*query_result[2])
        return ros_pose

    def parse_shelf_order_yaml(self):
        try:
            self.path_to_json = rospy.get_param('~order_yaml')
            with open(self.path_to_json, 'r') as f:
                order_dict = ordered_load(f, yaml.SafeLoader)
            self.order_dict = OrderedDict((self.get_shelf_system_from_erp_id(k), v) for k, v in order_dict.items())
            prev_id = None
            for i, shelf_system_id in enumerate(self.order_dict):
                if i > 0 and prev_id != self.order_dict[shelf_system_id]['starting-point']:
                    rospy.logwarn('starting point doesn\'t match the prev entry at {}'.format(shelf_system_id))
                prev_id = self.get_erp_id_from_shelf(shelf_system_id)
                via_points = self.order_dict[shelf_system_id]['via-points']
                for i in range(len(via_points)):
                    via_points[i] = convert_dictionary_to_ros_message("geometry_msgs/PoseStamped", via_points[i])
        except Exception as e:
            rospy.logwarn(e)
            rospy.loginfo('failed to load left right json, if you don\'t know what that means, you prob don\'t mind.')

    def delete_graph(self, name):
        q = 'tripledb:tripledb_graph_drop(\'{}\')'.format(name)
        return self.once(q) != []

    def is_left(self, shelf_system_id):
        return self.order_dict[shelf_system_id]['side'] == 'left'

    def is_right(self, shelf_system_id):
        return self.order_dict[shelf_system_id]['side'] == 'right'

    def get_shelf_system_ids(self, filter_with_left_right_dict=True):
        """
        :return: list of str
        :rtype: list
        """
        all_ids = set(self.get_all_individuals_of(SHELF_SYSTEM))
        if filter_with_left_right_dict:
            return [x for x in self.order_dict.keys() if x in all_ids]
        else:
            return all_ids

    def get_shelf_pose(self, shelf_system_id):
        return lookup_pose("map", self.get_object_frame_id(shelf_system_id))

    def get_num_of_tiles(self, shelf_system_id):
        if self.is_5tile_system(shelf_system_id):
            return 5
        elif self.is_6tile_system(shelf_system_id):
            return 6
        elif self.is_7tile_system(shelf_system_id):
            return 7
        else:
            raise Exception('Could not identify number of tiles for shelf {}.'.format(shelf_system_id))

    def is_5tile_system(self, shelf_system_id):
        q = 'instance_of(\'{}\', {})'.format(shelf_system_id, SHELF_T5)
        return self.once(q) == {}

    def is_heavy_system(self, shelf_system_id):
        q = 'instance_of(\'{}\', {})'.format(shelf_system_id, SHELF_H)
        return self.once(q) == {}

    def is_6tile_system(self, shelf_system_id):
        q = 'instance_of(\'{}\', {})'.format(shelf_system_id, SHELF_T6)
        return self.once(q) == {}

    def is_7tile_system(self, shelf_system_id):
        q = 'instance_of(\'{}\', {})'.format(shelf_system_id, SHELF_T7)
        return self.once(q) == {}

    def get_bottom_layer_type(self, shelf_system_id):
        q = 'shelf_bottom_floor_type(\'{}\', LayerType).'.format(shelf_system_id)
        return self.once(q)['LayerType']

    def get_shelf_layer_type(self, shelf_system_id):
        q = 'shelf_floor_type(\'{}\', LayerType).'.format(shelf_system_id)
        return self.once(q)['LayerType']

    def rename_graph(self, old_name, new_name):
        q = 'mng_update(roslog,triples,[graph,string({})],[\'$set\',[graph,string({})]])'.format(old_name, new_name)
        self.once(q)
        q = 'tripledb_add_subgraph({},common)'.format(new_name)
        self.once(q)
        q = 'tripledb_add_subgraph(user,{})'.format(new_name)
        self.once(q)

    def get_shelf_layer_from_system(self, shelf_system_id):
        """
        :type shelf_system_id: str
        :return: returns dict mapping floor id to pose ordered from lowest to highest
        :rtype: dict
        """
        q = 'triple(\'{}\', dul:hasComponent, Floor), ' \
            'instance_of(Floor, {}), ' \
            'object_feature_type(Floor, Feature, dmshop:\'DMShelfPerceptionFeature\'),' \
            'holds(Feature, knowrob:frameName, FeatureFrame).'.format(shelf_system_id, SHELF_FLOOR)

        solutions = self.all_solutions(q)
        floors = {}
        shelf_frame_id = self.get_perceived_frame_id(shelf_system_id)
        for solution in solutions:
            floor_id = solution['Floor'].replace('\'', '')
            floor_pose = lookup_pose(shelf_frame_id, solution['FeatureFrame'].replace('\'', ''))
            floors[floor_id] = floor_pose
        floors = floors.items()
        floors = list(sorted(floors, key=lambda x: x[1].pose.position.z))
        floors = [x for x in floors if x[1].pose.position.z < MAX_SHELF_HEIGHT]
        self.floors = OrderedDict(floors)
        return self.floors

    def get_object_of_facing(self, facing_id):
        q = 'shelf_facing_product_type(\'{}\', P)'.format(facing_id)
        solutions = self.all_solutions(q)
        if solutions:
            return solutions[0]['P'].replace('\'', '')

    def get_object_dimensions(self, object_class):
        """
        :param object_class:
        :return: [x length/depth, y length/width, z length/height]
        """
        q = 'object_dimensions(\'{}\', X_num, Y_num, Z_num).'.format(object_class)
        solutions = self.once(q)
        if solutions:
            return [solutions['Y_num'], solutions['X_num'], solutions['Z_num']]

    def assert_shelf_markers(self, left_pose, right_pose, left_id, right_id, shelf_pose):
        q = "belief_shelf_left_marker_at({}, '{}', Left)," \
            "belief_shelf_right_marker_at({}, '{}', Right).".format(
            self.pose_to_prolog(left_pose), left_id,
            self.pose_to_prolog(right_pose), right_id)
        bindings = self.once(q)
        # rospy.sleep(3)
        # self.
        # q = "show_marker(['{}', '{}'])".format(bindings['Left'], bindings['Right'])
        # q = "show_markers({})".format(rospy.get_rostime())
        q = "marker_plugin:republish."
        self.once(q)

        # assert shelf individual
        # self.belief_at_update()
        q = "belief_shelf_at('{}','{}',Shelf)".format(bindings['Left'], bindings['Right'])
        bindings = self.once(q)
        # bindings = self.once(q) # why do i have to call this twice?
        q = "tell(is_at(\'{}\', {})).".format(bindings['Shelf'], self.pose_to_prolog(shelf_pose))
        bindings = self.once(q)

    def republish_tf(self):
        time = rospy.get_rostime()
        frame_names = set()
        for i in range(10):
            q = 'holds(X, knowrob:frameName, Frame), has_type(X, O), transitive(subclass_of(O, dul:\'Object\')).'
            bindings = self.all_solutions(q)
            frame_names_tmp = set()
            for binding in bindings:
                frame_names.add(str(binding['Frame']))
            if len(frame_names_tmp) > len(frame_names):
                frame_names = frame_names_tmp
        q = 'forall( member(Frame, {0}), ' \
            '(tf_mng_lookup(Frame, _, {1}.{2}, P, _,_), ' \
            'tf_mem_set_pose(Frame, P, {1}.{2}),!)).'.format(list(frame_names), time.secs, time.nsecs)
        print q
        bindings = self.once(q)
        self.republish_marker()

    def republish_marker(self):
        q = 'marker_plugin:republish'
        self.once(q)

    def get_facing_ids_from_layer(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :return:
        :rtype: OrderedDict
        """
        shelf_system_id = self.get_shelf_system_from_layer(shelf_layer_id)
        q = 'findall([F, P], (shelf_facing(\'{}\', F),is_at(F, P)), Fs).'.format(shelf_layer_id)
        solutions = self.all_solutions(q)[0]
        facings = []
        for facing_id, pose in solutions['Fs']:
            facing_pose = self.prolog_to_pose_msg(pose)
            facing_pose = transform_pose(self.get_perceived_frame_id(shelf_layer_id), facing_pose)
            facings.append((facing_id, facing_pose))
        is_left = 1 if self.is_left(shelf_system_id) else -1
        facings = list(sorted(facings, key=lambda x: x[1].pose.position.x * is_left))
        return OrderedDict(facings)

    def get_label_ids(self, layer_id):
        """
        Returns the KnowRob IDs of all labels on one shelf layer.
        :param layer_id: KnowRob ID of the shelf for which the labels shall be retrieved.
        :type layer_id: str
        :return: KnowRob IDs of the labels on the shelf layer.
        :rtype: list
        """
        q = 'findall([L, X], (triple(\'{}\', dul:hasComponent, L), instance_of(L, dmshop:\'DMShelfLabel\'), is_at(L, [\'{}\' ,[X,_,_], _])), Ls).'.format(
            layer_id, layer_id)
        solutions = self.all_solutions(q)[0]
        sorted_solutions = list(sorted(solutions['Ls'], key=lambda x: x[1]))
        return [solution[0] for solution in sorted_solutions]

    def get_label_dan(self, label_id):
        """
        Returns the DAN of a label.
        :param label_id: KnowRob ID of the label for which the DAN shall be retrieved.
        :type label_id: str
        :return: DAN of the label.
        :rtype: str
        """
        q = 'triple(\'{}\', shop:articleNumberOfLabel, _AN), triple(_AN, shop:dan, DAN).'.format(label_id)
        solution = self.once(q)
        return solution['DAN'][1:-1]

    def get_label_pos(self, label_id):
        """
        Returns the 1-D position of a label, relative to the left edge of its shelf layer.
        :param label_id: KnowRob ID of the label for which to get the 1-D position.
        :type label_id: str
        :return: 1-D position of the label, relative to the left edge of its shelf layer (in m).
        :rtype: float
        """
        q = 'triple(_Layer, dul:hasComponent, \'{}\'), instance_of(_Layer, shop:\'ShelfLayer\'), is_at(\'{}\', [_Layer,[Pos,_,_],_]), object_dimensions(_Layer, _, Width, _).'.format(
            label_id, label_id)
        solution = self.once(q)
        return solution['Pos'] + solution['Width'] / 2.0

    def read_labels(self):
        """
        Reads and returns all label information in the belief state.
        :return: Read label information, ready for export.
        :rtype: list
        """
        labels = []
        for shelf_id in self.get_shelf_system_ids(filter_with_left_right_dict=False):
            for layer_num, layer_id in enumerate(self.get_shelf_layer_from_system(shelf_id).keys()):
                for label_num, label_id in enumerate(self.get_label_ids(layer_id)):
                    labels.append({
                        "label_num": label_num + 1,
                        "shelf_id": shelf_id,
                        "layer_num": layer_num + 1,
                        "dan": self.get_label_dan(label_id),
                        "pos": self.get_label_pos(label_id)})
        return labels

    def shelf_system_exists(self, shelf_system_id):
        """
        :type shelf_system_id: str
        :rtype: bool
        """
        return shelf_system_id in self.get_shelf_system_ids()

    def shelf_layer_exists(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :rtype: bool
        """
        q = 'shelf_layer_frame(\'{}\', _).'.format(shelf_layer_id)
        return self.once(q) == {}

    def facing_exists(self, facing_id):
        """
        :type facing_id: str
        :rtype: bool
        """
        q = 'shelf_facing(L, \'{}\').'.format(facing_id)
        return len(self.all_solutions(q)) != 0

    def get_facing_depth(self, facing_id):
        q = 'comp_facingDepth(\'{}\', W_XSD),atom_number(W_XSD,W)'.format(facing_id)
        solutions = self.once(q)
        if solutions:
            return solutions['W']
        raise Exception('can\'t compute facing depth')

    def get_facing_separator(self, facing_id):
        q = 'triple(\'{}\', shop:leftSeparator, L), triple(\'{}\', shop:rightSeparator, R)'.format(facing_id,
                                                                                                   facing_id)
        solutions = self.once(q)
        if solutions:
            return solutions['L'].replace('\'', ''), solutions['R'].replace('\'', '')

    def get_facing_height(self, facing_id):
        q = 'comp_facingHeight(\'{}\',W_XSD),atom_number(W_XSD,W)'.format(facing_id)
        solutions = self.once(q)
        if solutions:
            return solutions['W']
        raise Exception('can\' compute facing height')

    def get_facing_width(self, facing_id):
        q = 'comp_facingWidth(\'{}\', W_XSD),atom_number(W_XSD,W)'.format(facing_id)
        solutions = self.once(q)
        if solutions:
            return solutions['W']
        raise Exception('can\' compute facing width')

    def compute_shelf_product_type(self, facing_id):
        q = 'compute_shelf_facing_product_type(Facing, P)'.format(facing_id)
        self.once(q)

    def belief_at_update(self, id, pose):
        """
        :type id: str
        :type pose: PoseStamped
        """
        frame_id = self.get_object_frame_id(id)
        q = "get_time(T), " \
            "tf_mem_set_pose('{0}', {1}, T), " \
            "tf_mng_store('{0}', {1}, T)".format(frame_id,
                                                 self.pose_to_prolog(pose))
        return self.once(q)
        # q = 'is_at(\'{}\', {})'.format(id, self.pose_to_prolog(pose))
        # return self.once(q)

    # def get_objects(self, object_type):
    #     """
    #     Ask knowrob for a specific type of objects
    #     :type object_type: str
    #     :return: all objects of the given type
    #     :rtype: dict
    #     """
    #     objects = OrderedDict()
    #     q = 'instance_of(R, {}).'.format(object_type)
    #     solutions = self.all_solutions(q)
    #     for solution in solutions:
    #         object_id = solution['R'].replace('\'', '')
    #         pose_q = 'belief_at(\'{}\', R).'.format(object_id)
    #         believed_pose = self.once(pose_q)['R']
    #         ros_pose = PoseStamped()
    #         ros_pose.header.frame_id = believed_pose[0]
    #         ros_pose.pose.position = Point(*believed_pose[2])
    #         ros_pose.pose.orientation = Quaternion(*believed_pose[3])
    #         objects[str(object_id)] = ros_pose
    #     return objects

    def get_all_individuals_of(self, object_type):
        q = ' findall(R, instance_of(R, {}), Rs).'.format(object_type)
        solutions = self.once(q)['Rs']
        return [self.remove_quotes(solution) for solution in solutions]

    def remove_quotes(self, s):
        return s.replace('\'', '')

    # def belief_at(self, object_id):
    #     pose_q = 'belief_at(\'{}\', R).'.format(object_id)
    #     believed_pose = self.once(pose_q)['R']
    #     ros_pose = PoseStamped()
    #     ros_pose.header.frame_id = believed_pose[0]
    #     ros_pose.pose.position = Point(*believed_pose[2])
    #     ros_pose.pose.orientation = Quaternion(*believed_pose[3])
    #     return ros_pose

    def get_perceived_frame_id(self, object_id):
        """
        :type object_id: str
        :return: the frame_id of an object according to the specifications in our wiki.
        :rtype: str
        """
        if object_id not in self.perceived_frame_id_map:
            q = 'object_feature_type(\'{}\', Feature, dmshop:\'DMShelfPerceptionFeature\'),' \
                'holds(Feature, knowrob:frameName, FeatureFrame), !.'.format(
                object_id)
            self.perceived_frame_id_map[object_id] = self.once(q)['FeatureFrame'].replace('\'', '')
        return self.perceived_frame_id_map[object_id]

    def get_object_frame_id(self, object_id):
        """
        :type object_id: str
        :return: frame_id of the center of mesh.
        :rtype: str
        """
        q = 'holds(\'{}\', knowrob:frameName, R).'.format(object_id)
        return self.once(q)['R'].replace('\'', '')

    # floor
    def add_shelf_layers(self, shelf_system_id, shelf_layer_heights):
        """
        :param shelf_system_id: layers will be attached to this shelf system.
        :type shelf_system_id: str
        :param shelf_layer_heights: heights of the detects layers, list of floats
        :type shelf_layer_heights: list
        :return: TODO
        :rtype: bool
        """
        shelf_layer_heights = merge_close_shelf_layers(shelf_layer_heights)
        for i, height in enumerate(sorted(shelf_layer_heights)):
            if i == 0:
                layer_type = self.get_bottom_layer_type(shelf_system_id)
            else:
                if self.order_dict[shelf_system_id]['hack']:
                    shelf_layer = self.get_shelf_layer_type(shelf_system_id)
                    depth_id = shelf_layer.find('DMFloorT') + 8
                    old_depth = int(shelf_layer[depth_id])
                    new_depth = old_depth + 1
                    layer_type = shelf_layer.replace('DMFloorT{}'.format(old_depth), 'DMFloorT{}'.format(new_depth))
                else:
                    layer_type = self.get_shelf_layer_type(shelf_system_id)
            q = 'belief_shelf_part_at(\'{}\', \'{}\', {}, R)'.format(shelf_system_id, layer_type, height)
            self.once(q)
        return True

    def update_shelf_layer_position(self, shelf_layer_id, separators):
        """
        :type shelf_layer_id: str
        :type separators: list of PoseStamped, positions of separators
        """
        if len(separators) > 0:
            old_p = lookup_pose('map', self.get_perceived_frame_id(shelf_layer_id))
            separator_zs = [p.pose.position.z for p in separators]
            new_floor_height = np.mean(separator_zs)
            current_floor_pose = lookup_pose(MAP, self.get_object_frame_id(shelf_layer_id))
            current_floor_pose.pose.position.z += new_floor_height - old_p.pose.position.z
            shelf_system = self.get_shelf_system_from_layer(shelf_layer_id)
            frame_id = self.get_object_frame_id(shelf_system)
            current_floor_pose.header.stamp = rospy.Time()
            current_floor_pose = transform_pose(frame_id, current_floor_pose)
            q = 'tell(is_at(\'{}\', {}))'.format(shelf_layer_id, self.pose_to_prolog(current_floor_pose))
            self.once(q)

    def add_separators(self, shelf_layer_id, separators):
        """
        :param shelf_layer_id: separators will be attached to this shelf layer.
        :type shelf_layer_id: str
        :param separators: list of PoseStamped, positions of separators
        :return:
        """
        # TODO check success
        for p in separators:
            q = 'belief_shelf_part_at(\'{}\', {}, {}, _)'.format(shelf_layer_id, SEPARATOR, p.pose.position.x)
            try:
                self.once(q)
            except Exception as e:
                traceback.print_exc()
                return False
        return True

    def add_barcodes(self, shelf_layer_id, barcodes):
        """
        :param shelf_layer_id: barcodes will be attached to this shelf layer
        :type shelf_layer_id: str
        :param barcodes: dict mapping barcode to PoseStamped. make sure it relative to shelf layer, everything but x ignored
        :type barcodes: dict
        """
        # TODO check success
        for barcode, p in barcodes.items():
            if not self.does_DAN_exist(barcode):
                q = 'create_article_number(dan(\'{}\'),AN), ' \
                    'create_article_type(AN,[{},{},{}],ProductType).'.format(barcode, 0.4, 0.015, 0.1)
                r = self.once(q)
            q = 'belief_shelf_barcode_at(\'{}\', {}, dan(\'{}\'), {}, _).'.format(shelf_layer_id, BARCODE,
                                                                                  barcode, p.pose.position.x)
            self.once(q)

    def create_unknown_barcodes(self, barcodes):
        for barcode, p in barcodes.items():
            if not self.does_DAN_exist(barcode):
                q = 'create_article_number(dan(\'{}\'),AN), ' \
                    'create_article_type(AN,[{},{},{}],ProductType).'.format(barcode, 0.4, 0.015, 0.1)
                r = self.once(q)

    def add_separators_and_barcodes(self, shelf_layer_id, separators, barcodes):
        t = lookup_transform(self.get_perceived_frame_id(shelf_layer_id), 'map')
        separators = [do_transform_pose(p, t) for p in separators]
        barcodes = {code: do_transform_pose(p, t) for code, p in barcodes.items()}
        shelf_layer_width = self.get_shelf_layer_width(shelf_layer_id)
        separators_xs = [p.pose.position.x / shelf_layer_width for p in separators]
        barcodes = [(p.pose.position.x / shelf_layer_width, barcode) for barcode, p in barcodes.items()]

        # definitely no hacks here
        separators_xs, barcodes = add_separator_between_barcodes(separators_xs, barcodes)
        separators_xs = add_edge_separators(separators_xs)
        separators_xs = merge_close_separators(separators_xs)

        q = 'bulk_insert_floor(\'{}\', separators({}), labels({}))'.format(shelf_layer_id, separators_xs, barcodes)
        self.once(q)
        rospy.sleep(5)
        self.republish_marker()
        # q = 'shelf_facings_mark_dirty(\'{}\')'.format(shelf_layer_id)
        # self.once(q)

    # def assert_confidence(self, facing_id, confidence):
    #     q = 'tell(holds(\'{}\', knowrob:confidence, \'{}\')).'.format(facing_id, confidence)
    #     self.once(q)

    def does_DAN_exist(self, dan):
        q = 'article_number_of_dan(\'{}\', _)'.format(dan)
        return self.once(q) == {}

    def get_all_product_dan(self):
        """
        :return: list of str
        :rtype: list
        """
        # this force knowrob to load everything
        q = 'findall(DAN, triple(AN, shop:dan, DAN), DANS).'
        dans = self.once(q)['DANS']
        q = 'findall(DAN, triple(AN, shop:gtin, DAN), DANS)'
        self.once(q)
        return dans

    def init_product_gtin_map(self):
        while True:
            q = 'findall([P, G], product_to_gtin(P, G), L)'
            l = self.once(q)['L']
            if len(l) != 1817:
                rospy.logwarn('got {} instead of 1817 products. trying again'.format(len(l)))
            else:
                break
        self.product_to_gtin = {p:g for p,g in l}
        self.gtin_to_product = {g:p for p,g in l}

    def add_objects(self, facing_id, number, gtin):
        """
        Adds objects to the facing whose type is according to the barcode.
        :type facing_id: str
        :type number: int
        """
        for i in range(number):
            if gtin in self.gtin_to_product:
                product = self.gtin_to_product[gtin]
                q = 'product_spawn_front_to_back(\'{}\', ObjId, \'{}\')'.format(facing_id, product)
            else:
                q = 'product_spawn_front_to_back(\'{}\', ObjId)'.format(facing_id)
            self.once(q)

    def save_beliefstate(self, path=None):  ### beleifstate.owl might not be created. the data is stored in tripledb
        """
        :type path: str
        """
        self.stop_episode(path)
        # pass
        # if path is None:
        #     path = '{}/data/beliefstate.owl'.format(RosPack().get_path('refills_second_review'))
        # q = 'memorize(\'{}\')'.format(path)
        # self.once(q)

    def get_shelf_layer_width(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :rtype: float
        """
        q = 'object_dimensions(\'{}\', _, W, _)'.format(shelf_layer_id)
        solution = self.once(q)
        if solution:
            width = solution['W']
            return width
        else:
            raise Exception('width not defined for {}'.format(shelf_layer_id))

    def get_shelf_system_width(self, shelf_system_id):
        """
        :type shelf_system_id: str
        :rtype: float
        """
        q = 'object_dimensions(\'{}\', _, W, _)'.format(shelf_system_id)
        solution = self.once(q)
        width = solution['W']
        return width

    def get_shelf_system_height(self, shelf_system_id):
        """
        :type shelf_system_id: str
        :rtype: float
        """
        q = 'object_dimensions(\'{}\', _, _, H)'.format(shelf_system_id)
        solution = self.once(q)
        height = solution['H']
        return height

    def get_all_empty_facings(self):
        q = 'findall(Facing, (has_type(Facing, shop:\'ProductFacingStanding\'),\+holds(Facing, shop:productInFacing,_)),Fs)'
        solution = self.once(q)
        if solution:
            return solution['Fs']
        return []

    def get_empty_facings_from_layer(self, shelf_layer_id):
        q = 'findall(F, (shelf_facing(\'{}\', F), \+holds(F, shop:productInFacing, _)),Fs)'.format(shelf_layer_id)
        solution = self.once(q)
        if solution:
            return solution['Fs']
        return []

    def get_shelf_system_from_layer(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :rtype: str
        """
        if shelf_layer_id not in self.shelf_system_from_layer:
            q = 'shelf_layer_frame(\'{}\', Frame).'.format(shelf_layer_id)
            shelf_system_id = self.once(q)['Frame']
            self.shelf_system_from_layer[shelf_layer_id] = shelf_system_id
        return self.shelf_system_from_layer[shelf_layer_id]

    def get_shelf_layer_from_facing(self, facing_id):
        """
        :type facing_id: str
        :rtype: str
        """
        if facing_id not in self.shelf_layer_from_facing:
            q = 'shelf_facing(Layer, \'{}\').'.format(facing_id)
            layer_id = self.once(q)['Layer']
            self.shelf_layer_from_facing[facing_id] = layer_id
        return self.shelf_layer_from_facing[facing_id]

    def get_shelf_layer_above(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :return: shelf layer above or None if it does not exist.
        :rtype: str
        """
        q = 'shelf_layer_above(\'{}\', Above).'.format(shelf_layer_id)
        solution = self.once(q)
        if isinstance(solution, dict):
            return solution['Above']

    def get_shelf_system_from_erp_id(self, id):
        q = 'shelf_with_erp_id(Shelf, {})'.format(id)
        bindings = self.once(q)
        try:
            return bindings['Shelf']
        except TypeError:
            raise PrologException('no shelf found with id {}'.format(id))

    def get_erp_id_from_shelf(self, shelf_id):
        q = 'shelf_with_erp_id(\'{}\', ID)'.format(shelf_id)
        bindings = self.once(q)
        try:
            return bindings['ID']
        except TypeError:
            raise PrologException('no id found for shelf {}'.format(shelf_id))

    def is_top_layer(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :return: shelf layer above or None if it does not exist.
        :rtype: str
        """
        return self.get_shelf_layer_above(shelf_layer_id) is None

    def is_bottom_layer(self, shelf_layer_id):
        """
        :type shelf_layer_id: str
        :return: shelf layer above or None if it does not exist.
        :rtype: str
        """
        q = 'instance_of(\'{}\', {})'.format(shelf_layer_id, SHELF_BOTTOM_LAYER)
        return self.once(q) != []

    # Neem logging
    # helper to create an action
    def neem_create_action(self):
        q = 'tell(is_action(Act))'
        solutions = self.all_solutions(q)
        if solutions:
            return solutions[0]

    # initialize
    def neem_init(self, robot_iri, store_iri):
        q = 'tf_logger_enable,'\
            'tripledb_load(\'package://knowrob_refills/owl/iai-shop.owl\'),'\
            'tripledb_load(\'package://knowrob/owl/robots/IIWA.owl\'),'\
            'urdf_load(\'{}\', \'package://knowrob/urdf/iiwa.urdf\', [load_rdf]),'\
            'tell([is_episode(Episode),'\
            'is_setting_for(Episode,\'{}\'),'\
            'is_setting_for(Episode,\'{}\')'\
            '])'.format(robot_iri,store_iri,robot_iri,store_iri)
        solutions = self.all_solutions(q)
        if solutions:
            return solutions[0]

    # stocktaking
    def neem_stocktacking(self, act_iri, store_iri, robot_iri, begin_act, end_act, episode_iri):
        q = 'Act = \'{}\',tell(['\
            'has_participant(Act,\'{}\'),'\
            'is_performed_by(Act,\'{}\'),'\
            'occurs(Act) during [\'{}\',\'{}\'],'\
            'has_type(Tsk,shop:\'Stocktaking\'),'\
            'has_type(Role,soma:\'Location\'),'\
            'has_task_role(Tsk,Role),'\
            'executes_task(Act,Tsk),'\
            'has_role(\'{}\',Role) during Act,'\
            'is_setting_for(\'{}\',Act)'\
            '])'.format(act_iri, store_iri, robot_iri, begin_act, end_act, store_iri, episode_iri)
        solutions = self.all_solutions(q)
        if solutions:
            return solutions[0]

    # a
    def neem_park_arm(self, robot_arm_iri, robot_iri, begin_act, end_act, parent_act_iri):
        q = 'tell(is_action(Act)),'\
        'notify_synchronize(event(A)),'\
        'tell(['\
        'has_participant(Act,\'{}\'),'\
        'is_performed_by(Act,\'{}\'),'\
        'occurs(Act) during [\'{}\',\'{}\'],'\
        'has_type(Tsk,soma:\'ParkingArms\'),'\
        'has_type(Role,soma:\'MovedObject\'),'\
        'has_task_role(Tsk,Role),'\
        'has_role(\'{}\',Role) during Act,'\
        'executes_task(Act,Tsk),'\
        'has_type(Mot,soma:\'LimbMotion\'),'\
        'is_classified_by(Act,Mot),'\
        'has_process_role(Mot,Role),'\
        'has_subevent(\'{}\',Act)'\
        '])'.format(robot_arm_iri, robot_iri, begin_act, end_act, robot_arm_iri, parent_act_iri)
        solutions = self.all_solutions(q)
        if solutions:
            return solutions[0]

    # b, c1, d2
    def neem_navigate_to_shelf(self, shelve_row_iri, robot_iri, begin_act, end_act, parent_act_iri):
        q = 'tell(is_action(Act)),'\
            'notify_synchronize(event(A)),'\
            'tell(['\
            'is_action(Act),'\
            'has_participant(Act,\'{}\'),'\
            'is_performed_by(Act,\'{}\'),'\
            'occurs(Act) during [\'{}\',\'{}\'],'\
            'has_type(Tsk,soma:\'MovingTo\'),'\
            'has_type(Role,soma:\'Destination\'),'\
            'executes_task(Act,Tsk),'\
            'has_task_role(Tsk,Role),'\
            'has_type(Mot,soma:\'Driving\'),'\
            'is_classified_by(Act,Mot),'\
            'has_role(\'{}\',Role) during Act,'\
            'has_subevent(\'{}\',Act)'\
        '])'.format(shelve_row_iri, robot_iri, begin_act, end_act,shelve_row_iri, parent_act_iri)
        solutions = self.all_solutions(q)
        if solutions:
            return solutions[0]

    # c, d, e
    def neem_for_shelf(self, act_iri, shelve_iri, robot_iri, begin_act, end_act, parent_act_iri):
        q = 'Act = \'{}\',tell(['\
            'has_participant(Act,\'{}\'),'\
            'is_performed_by(Act,\'{}\'),'\
            'occurs(Act) during [\'{}\',\'{}\'],'\
            'has_type(Tsk,shop:\'Stocktaking\'),'\
            'has_type(Role,soma:\'Location\'),'\
            'has_task_role(Tsk,Role),'\
            'executes_task(Act,Tsk),'\
            'has_role(\'{}\',Role) during Act,'\
            'has_subevent(\'{}\',Act)'\
        '])'.format(act_iri, shelve_iri, robot_iri, begin_act, end_act, shelve_iri, parent_act_iri)
        solutions = self.all_solutions(q)
        if solutions:
            return solutions[0]

    # c2
    def neem_camera_initial_scan_pose(self, robot_arm_iri, robot_iri, begin_act, end_act, parent_act_iri):
        q = 'tell(is_action(Act)),'\
        'notify_synchronize(event(Act)),'\
        'tell(['\
        'has_participant(Act,\'{}\'),'\
        'is_performed_by(Act,\'{}\'),'\
        'occurs(Act) during [\'{}\',\'{}\'],'\
        'has_type(Tsk,soma:\'AssumingArmPose\'),'\
        'has_type(Role,soma:\'MovedObject\'),'\
        'has_task_role(Tsk,Role),'\
        'has_role(\'{}\',Role) during Act,'\
        'executes_task(Act,Tsk),'\
        'has_type(Mot,soma:\'LimbMotion\'),'\
        'is_classified_by(Act,Mot),'\
        'has_process_role(Mot,Role),'\
        'has_subevent(\'{}\',Act)'\
        '])'.format(robot_arm_iri, robot_iri, begin_act, end_act, robot_arm_iri, parent_act_iri)
        solutions = self.all_solutions(q)
        if solutions:
            return solutions[0]

    # c3
    def neem_navigate_to_middle_of_shelf(self, shelve_row_iri, robot_iri, begin_act, end_act, parent_act_iri):
        q = 'tell(is_action(Act)),'\
            'notify_synchronize(event(A)),'\
            'tell(['\
            'has_participant(Act,\'{}\'),'\
            'is_performed_by(Act,\'{}\'),'\
            'occurs(Act) during [\'{}\',\'{}\'],'\
            'has_type(Tsk,soma:\'Navigation\'),'\
            'has_type(Role,soma:\'Destination\'),'\
            'executes_task(Act,Tsk),'\
            'has_task_role(Tsk,Role),'\
            'has_type(Mot,soma:\'Driving\'),'\
            'is_classified_by(Act,Mot),'\
            'has_role(\'{}\',Role) during Act,'\
            'has_subevent(\'{}\',Act)'\
        '])'.format(shelve_row_iri, robot_iri, begin_act, end_act,shelve_row_iri, parent_act_iri)
        solutions = self.all_solutions(q)
        if solutions:
            return solutions[0]

    # c4
    def neem_move_camera_top_to_bottom(self, shelve_iri, robot_iri, robot_arm_iri, begin_act, end_act, parent_act_iri):
        q = 'tell(is_action(Act)),'\
            'notify_synchronize(event(A)),'\
            'tell(['\
           'has_participant(Act,\'{}\'),'\
           'is_performed_by(Act,\'{}\'),'\
           'occurs(Act) during [\'{}\',\'{}\'],'\
           'has_type(Tsk,soma:\'LookingAt\'),'\
           'executes_task(Act,Tsk),'\
           'has_type(Role1,soma:\'Location\'),'\
           'has_task_role(Tsk,Role1),'\
           'has_role(\'{}\',Role1) during Act,'\
           'has_type(Mot,soma:\'LimbMotion\'),'\
           'is_classified_by(Act,Mot),'\
           'has_type(Role2,soma:\'MovedObject\'),'\
           'has_process_role(Mot,Role2),'\
           'has_role(\'{}\',Role2) during Act,'\
           'has_subevent(\'{}\',Act)'\
           '])'.format(shelve_iri, robot_iri, begin_act, end_act,shelve_iri,robot_arm_iri,parent_act_iri)
        solutions = self.all_solutions(q)
        if solutions:
            return solutions[0]

    # d1, e1
    def neem_position_camera_floor(self, robot_arm_iri, robot_iri, begin_act, end_act, parent_act_iri):
        q = 'tell(is_action(Act)),'\
        'notify_synchronize(event(A)),'\
        'tell(['\
        'has_participant(Act,\'{}\'),'\
        'is_performed_by(Act,\'{}\'),'\
        'occurs(Act) during [\'{}\',\'{}\'],'\
        'has_type(Tsk,soma:\'AssumingArmPose\'),'\
        'has_type(Role,soma:\'MovedObject\'),'\
        'has_task_role(Tsk,Role),'\
        'has_role(\'{}\',Role) during Act,'\
        'executes_task(Act,Tsk),'\
        'has_type(Mot,soma:\'LimbMotion\'),'\
        'is_classified_by(Act,Mot),'\
        'has_process_role(Mot,Role),'\
        'has_subevent(\'{}\',Act)'\
        '])'.format(robot_arm_iri, robot_iri, begin_act, end_act, robot_arm_iri, parent_act_iri)
        solutions = self.all_solutions(q)
        if solutions:
            return solutions[0]

    # d3, e2
    def neem_navigate_aliong_shelf(self, shelve_floor_iri, robot_iri, begin_act, end_act, parent_act_iri):
        q = 'tell(is_action(Act)),'\
            'notify_synchronize(event(A)),'\
            'tell(['\
           'has_participant(Act,\'{}\'),'\
           'is_performed_by(Act,\'{}\'),'\
           'occurs(Act) during [\'{}\',\'{}\'],'\
           'has_type(Tsk,soma:\'LookingAt\'),'\
           'executes_task(Act,Tsk),'\
           'has_type(Role,soma:\'Location\'),'\
           'has_task_role(Tsk,Role),'\
           'has_role(\'{}\',Role) during Act,'\
           'has_type(Mot,soma:\'Navigation\'),'\
           'is_classified_by(Act,Mot),'\
          'has_subevent(\'{}\',Act)'\
        '])'.format(shelve_floor_iri, robot_iri, begin_act, end_act, shelve_floor_iri, parent_act_iri)
        solutions = self.all_solutions(q)
        if solutions:
            return solutions[0]


    # def clear_beliefstate(self, initial_beliefstate=None):
    #     """
    #     :rtype: bool
    #     """
    #     q = 'findall(Coll, (mng_collections(roslog,Coll), \+ Coll=\'system.indexes\'), L), forall(member(C, L), mng_drop(roslog,C)),' \
    #         'tripledb_load(\'package://knowrob/owl/knowrob.owl\',[graph(tbox),namespace(knowrob)]),' \
    #         'tripledb_init.'
    #         # 'tell([is_episode(Episode)]). '
    #     # 'is_action(Action), ' \
    #     # 'has_type(Task, soma:\'PhysicalTask\'),' \
    #     # 'executes_task(Action,Task),' \
    #     # 'is_setting_for(Episode,Action)]),' \
    #     # 'notify_synchronize(event(Action)),' \
    #     # '!.'
    #     result = self.once(q)
    #     if initial_beliefstate is None:
    #         initial_beliefstate = self.initial_beliefstate
    #     if initial_beliefstate is not None:
    #         q = 'remember({})'.format(initial_beliefstate)
    #         result = self.once(q)
    #         if result == []:
    #             raise RuntimeError('failed to load {}'.format(initial_beliefstate))

    # # put path of owl here
    # # q = 'retractall(owl_parser:owl_file_loaded(\'{}/beliefstate.owl\'))'.format(initial_beliefstate)
    # # Works only if the beliefstate.owl is loaded with namespace beliefstate
    # q = 'tripledb:tripledb_graph_drop(' + \
    #     'beliefstate)'.format(initial_beliefstate)
    # result = self.once(q) != []
    # self.reset_object_state_publisher.call(TriggerRequest())
    # return result

    # def reset_beliefstate(self, inital_beliefstate=None):
    #     """
    #     :rtype: bool
    #     """
    #     return self.load_initial_beliefstate()

    def mongo_load_database(self, path=None):
        if path is None:
            self.initial_beliefstate = rospy.get_param('~initial_beliefstate')
        else:
            self.initial_beliefstate = path
        print('loading {} into mongo'.format(path))
        # q = 'remember(\'{}\'),  tf_mng_remember(\'{}\').'.format(path, path)
        # self.once(q)
        cmd = 'mongorestore -d roslog {}'.format(path)
        rospy.loginfo('executing: {}'.format(cmd))
        os.system(cmd)

        # q = 'remember(\'{}\')'.format(self.initial_beliefstate)
        # result = self.once(q)
        # if result == []:
        #     raise RuntimeError('failed to load {}'.format(self.initial_beliefstate))
        # return True
        # self.clear_beliefstate(self.initial_beliefstate)
        # if self.start_episode(self.initial_beliefstate):
        #     print_with_prefix('loaded initial beliefstate {}'.format(self.initial_beliefstate), self.prefix)
        #     self.reset_object_state_publisher.call(TriggerRequest())
        #     return True
        # else:
        #     print_with_prefix('error loading initial beliefstate {}'.format(self.initial_beliefstate), self.prefix)
        #     return False

    # def load_owl(self, path):
    #     """
    #     :param pafh: path to log folder
    #     :type path: str
    #     :rtype: bool
    #     """
    #     q = 'tripledb_load(\'{}\')'.format(path + "/beliefstate.owl")
    #     return self.once(q) != []

    def start_episode(self):
        raise NotImplementedError()

    #     self.clear_beliefstate(path_to_old_episode)
    #     q = 'tell([is_episode(Episode)]).'
    #     result = self.once(q)
    # if path_to_old_episode is not None:
    #     q = 'remember({})'.format(path_to_old_episode)
    #     result = self.once(q)
    #     if result == []:
    #         raise RuntimeError('failed to load {}'.format(path_to_old_episode))
    # if result:
    #     q = 'knowrob_memory:current_episode(E), mem_episode_stop(E)'
    #     self.once(q)

    # if path_to_old_episode is None:
    #     q = 'mem_episode_start(E).'
    #     result = self.once(q)
    #     self.episode_id = result['E']
    # else:
    #     q = 'mem_episode_start(E, [import:\'{}\']).'.format(path_to_old_episode)
    #     result = self.once(q)
    #     self.episode_id = result['E']
    # return result != []

    def stop_episode(self):
        raise NotImplementedError()

    def mongo_drop_database(self, name):
        os.system('mongo {} --eval "db.dropDatabase()"'.format(name))

    def mongo_dump_database(self, path):
        # q = ''
        # q = 'get_time(CurrentTime), ' \
        #     'atom_concat(\'{}\',\'/\',X1), ' \
        #     'atom_concat(X1,CurrentTime,X2), ' \
        #     'memorize(X2).'.format(path)
        #     # 'findall(Coll, (mng_collection(roslog,Coll), \+ Coll=\'system.indexes\'), L), forall(member(C, L), mng_drop(roslog,C)).'.format(path)
        # result = self.once(q)
        # if result == []:
        #     raise RuntimeError('failed to store episode')
        # else:
        #     rospy.loginfo('saved episode at {}'.format(path))
        #     return True

        # print(os.getcwd())
        os.system('mongodump --db roslog --out {}'.format(path))
        # q = 'tf_mng_memorize(\'{}\'),  memorize(\'{}\').'.format(path, path)
        # self.once(q)

        # q = 'mem_episode_stop(\'{}\').'.format(self.episode_id)
        # return self.once(q) != []

    # def start_tf_logging(self):
    #     q = 'ros_logger_start([[\'tf\',[]]])'
    #     self.once(q)
    #
    # def stop_tf_logging(self):
    #     q = 'ros_logger_stop.'
    #     self.once(q)




if __name__ == u'__main__':
    rospy.init_node('perception_interface')
    kb = KnowRob()
    kb.once('1=0.')
