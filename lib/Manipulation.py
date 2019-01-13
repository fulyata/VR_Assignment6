#!/usr/bin/python

### import guacamole libraries ###
import avango
import avango.gua
import avango.script
from avango.script import field_has_changed
import avango.daemon

### import python libraries ###
import math
import sys
import time

class ManipulationManager(avango.script.Script):

    ## input fields
    sf_toggle_button = avango.SFBool()

    ## constructor
    def __init__(self):
        self.super(ManipulationManager).__init__()    
    
    
    def my_constructor(self,
        SCENEGRAPH = None,
        NAVIGATION_NODE = None,
        HEAD_NODE = None,
        POINTER_INPUT = None,
        ):

        
        ### variables ###
        self.active_manipulation_technique = None
        self.active_manipulation_technique_index = None
        self.sf_toggle_button.connect_from(POINTER_INPUT.sf_toggle_button)

    
        ## init manipulation techniques
        self.ray = Ray()
        self.ray.my_constructor(SCENEGRAPH, NAVIGATION_NODE, POINTER_INPUT)


        self.depthRay = DepthRay()
        self.depthRay.my_constructor(SCENEGRAPH, NAVIGATION_NODE, POINTER_INPUT)


        self.goGo = GoGo()
        self.goGo.my_constructor(SCENEGRAPH, NAVIGATION_NODE, HEAD_NODE, POINTER_INPUT)


        self.virtualHand = VirtualHand()
        self.virtualHand.my_constructor(SCENEGRAPH, NAVIGATION_NODE, POINTER_INPUT)

    
        ### set initial states ###
        self.set_manipulation_technique(0) # switch to virtual-ray manipulation technique



    ### functions ###
    def set_manipulation_technique(self, INT):
        # possibly disable prior technique
        if self.active_manipulation_technique is not None:
            self.active_manipulation_technique.enable(False)
    
        # enable new technique
        if INT == 0: # ray
            print("switch to Ray technique")
            self.active_manipulation_technique = self.ray

        elif INT == 1: # depth ray
            print("switch to Depth-Ray technique")
            self.active_manipulation_technique = self.depthRay

        elif INT == 2: # go-go
            print("switch to Go-Go technique")
            self.active_manipulation_technique = self.goGo

        elif INT == 3: # HOMER
            print("switch to Virtual-Hand (PRISM) technique")
            self.active_manipulation_technique = self.virtualHand

        self.active_manipulation_technique_index = INT
        self.active_manipulation_technique.enable(True)


    ### callback functions ###
    @field_has_changed(sf_toggle_button)
    def sf_toggle_button_changed(self):
        if self.sf_toggle_button.value == True: # key is pressed
            next_index = (self.active_manipulation_technique_index + 1) % 4
            self.set_manipulation_technique(next_index) # switch to Ray manipulation technique



class ManipulationTechnique(avango.script.Script):

    ## input fields
    sf_button = avango.SFBool()

    ## constructor
    def __init__(self):
        self.super(ManipulationTechnique).__init__()
               

    def my_constructor(self,
        SCENEGRAPH = None,
        NAVIGATION_NODE = None,
        POINTER_INPUT = None,
        ):


        ### external references ###
        self.SCENEGRAPH = SCENEGRAPH
        self.POINTER_INPUT = POINTER_INPUT
            
            
        ### variables ###
        self.enable_flag = False
        
        self.selected_node = None
        self.selected_nodes = []
        self.dragged_node = None
        self.dragging_offset_mat = avango.gua.make_identity_mat()
                
        self.mf_pick_result = []
        self.pick_result = None # chosen pick result
        self.white_list = []   
        self.black_list = ["invisible"]
        self.nodes_behind_selected_node = []

        #self.pick_options = avango.gua.PickingOptions.PICK_ONLY_FIRST_OBJECT \
        #                    | avango.gua.PickingOptions.PICK_ONLY_FIRST_FACE \
        #                    | avango.gua.PickingOptions.GET_POSITIONS \
        #                    | avango.gua.PickingOptions.GET_NORMALS \
        #                    | avango.gua.PickingOptions.GET_WORLD_POSITIONS \
        #                    | avango.gua.PickingOptions.GET_WORLD_NORMALS

        self.pick_options = avango.gua.PickingOptions.PICK_ONLY_FIRST_FACE \
                            | avango.gua.PickingOptions.GET_POSITIONS \
                            | avango.gua.PickingOptions.GET_NORMALS \
                            | avango.gua.PickingOptions.GET_WORLD_POSITIONS \
                            | avango.gua.PickingOptions.GET_WORLD_NORMALS



        ### resources ###

        ## init nodes
        self.pointer_node = avango.gua.nodes.TransformNode(Name = "pointer_node")
        self.pointer_node.Tags.value = ["invisible"]
        NAVIGATION_NODE.Children.value.append(self.pointer_node)
        

        self.ray = avango.gua.nodes.Ray() # required for trimesh intersection

        ## init field connections
        self.sf_button.connect_from(self.POINTER_INPUT.sf_button0)
        self.pointer_node.Transform.connect_from(self.POINTER_INPUT.sf_pointer_mat)
            
        self.always_evaluate(True) # change global evaluation policy


    ### functions ###
    def enable(self, BOOL):
        self.enable_flag = BOOL
        
        if self.enable_flag == True:
            self.pointer_node.Tags.value = [] # set tool visible
        else:
            self.stop_dragging() # possibly stop active dragging process
            
            self.pointer_node.Tags.value = ["invisible"] # set tool invisible

       
    def start_dragging(self, NODE):
        self.dragged_node = NODE        
        self.dragging_offset_mat = avango.gua.make_inverse_mat(self.pointer_node.WorldTransform.value) * self.dragged_node.WorldTransform.value # object transformation in pointer coordinate system

  
    def stop_dragging(self): 
        self.dragged_node = None
        self.dragging_offset_mat = avango.gua.make_identity_mat()


    def dragging(self):
        if self.dragged_node is not None: # object to drag
            _new_mat = self.pointer_node.WorldTransform.value * self.dragging_offset_mat # new object position in world coodinates
            _new_mat = avango.gua.make_inverse_mat(self.dragged_node.Parent.value.WorldTransform.value) * _new_mat # transform new object matrix from global to local space
        
            self.dragged_node.Transform.value = _new_mat


    def get_roll_angle(self, MAT4):
        #print(MAT4.get_rotate())
        _dir_vec = avango.gua.make_rot_mat(MAT4.get_rotate()) * avango.gua.Vec3(0.0,0.0,-1.0)
        #print(avango.gua.make_rot_mat(MAT4.get_rotate()) * avango.gua.Vec3(0.0,0.0,-1.0))
        _dir_vec = avango.gua.Vec3(_dir_vec.x, _dir_vec.y, _dir_vec.z) # cast to Vec3
        
        _ref_side_vec = avango.gua.Vec3(1.0,0.0,0.0)
   
        _up_vec = _dir_vec.cross(_ref_side_vec)
        _up_vec.normalize()
        _ref_side_vec = _up_vec.cross(_dir_vec)
        _ref_side_vec.normalize()      
        
        _side_vec = avango.gua.make_rot_mat(MAT4.get_rotate()) * avango.gua.Vec3(1.0,0.0,0.0)    
        _side_vec = avango.gua.Vec3(_side_vec.x, _side_vec.y, _side_vec.z) # cast to Vec3
        #print(_ref_side_vec, _side_vec)

        #_axis = _ref_side_vec.cross(_side_vec)
        #_axis.normalize()
    
        _angle = math.degrees(math.acos(min(max(_ref_side_vec.dot(_side_vec), -1.0), 1.0)))
        #print(_angle)
        
        if _side_vec.y > 0.0: # simulate rotation direction
            _angle *= -1.0
                
        return _angle



    def update_intersection(self, PICK_MAT = avango.gua.make_identity_mat(), PICK_LENGTH = 1.0):
        # update ray parameters
        self.ray.Origin.value = PICK_MAT.get_translate()

        _vec = avango.gua.make_rot_mat(PICK_MAT.get_rotate_scale_corrected()) * avango.gua.Vec3(0.0,0.0,-1.0)
        _vec = avango.gua.Vec3(_vec.x,_vec.y,_vec.z)

        self.ray.Direction.value = _vec * PICK_LENGTH

        ## trimesh intersection
        self.mf_pick_result = self.SCENEGRAPH.ray_test(self.ray, self.pick_options, self.white_list, self.black_list)



    # def update_intersection_DpR(self, PICK_MAT = avango.gua.make_identity_mat(), PICK_LENGTH = 1.0):
    #     # update ray parameters
    #     self.ray.Origin.value = PICK_MAT.get_translate()

    #     _vec = avango.gua.make_rot_mat(PICK_MAT.get_rotate_scale_corrected()) * avango.gua.Vec3(0.0,0.0,-1.0)
    #     _vec = avango.gua.Vec3(_vec.x,_vec.y,_vec.z)

    #     self.ray.Direction.value = _vec * PICK_LENGTH

    #     ## trimesh intersection
    #     self.mf_pick_result = self.SCENEGRAPH.ray_test(self.ray, self.pick_options, self.white_list, self.black_list)

    def highlight(self, node, color, color_override = True):
        if node is not None:
            for _child_node in node.Children.value:
                if _child_node.get_type() == 'av::gua::TriMeshNode':
                    _child_node.Material.value.set_uniform("enable_color_override", color_override)
                    _child_node.Material.value.set_uniform("override_color", color)

    def selection(self):
        if len(self.mf_pick_result.value) > 0: # intersection found
            self.pick_result = self.mf_pick_result.value[0] # get first pick result

        else: # nothing hit
            self.pick_result = None


        ## disable previous node highlighting
        if self.selected_node is not None:
            for _child_node in self.selected_node.Children.value:
                if _child_node.get_type() == 'av::gua::TriMeshNode':
                    _child_node.Material.value.set_uniform("enable_color_override", False)


        if self.pick_result is not None: # something was hit
            self.selected_node = self.pick_result.Object.value # get intersected geometry node
            self.selected_node = self.selected_node.Parent.value # take the parent node of the geomtry node (the whole object)

        else:
            self.selected_node = None

        ## enable node highlighting
        if self.selected_node is not None:
            for _child_node in self.selected_node.Children.value:
                if _child_node.get_type() == 'av::gua::TriMeshNode':
                    _child_node.Material.value.set_uniform("enable_color_override", True)
                    _child_node.Material.value.set_uniform("override_color", avango.gua.Vec4(1.0,0.0,0.0,0.3)) # 30% color override               

    ### callback functions ###

    @field_has_changed(sf_button)
    def sf_button_changed(self):
        if self.sf_button.value == True: # button pressed
            if self.selected_node is not None:
                self.start_dragging(self.selected_node)

        else: # button released
            self.stop_dragging()
            
            
    def evaluate(self): # evaluated every frame
        raise NotImplementedError("To be implemented by a subclass.")
            
            

class Ray(ManipulationTechnique):

    ## constructor
    def __init__(self):
        self.super(Ray).__init__()


    def my_constructor(self,
        SCENEGRAPH = None,
        NAVIGATION_NODE = None,
        POINTER_INPUT = None,
        ):

        ManipulationTechnique.my_constructor(self, SCENEGRAPH, NAVIGATION_NODE, POINTER_INPUT) # call base-class constructor


        ### parameters ###
        self.ray_length = 2.5 # in meter
        self.ray_thickness = 0.01 # in meter

        self.intersection_point_size = 0.03 # in meter


        ### resources ###
        _loader = avango.gua.nodes.TriMeshLoader()

        self.ray_geometry = _loader.create_geometry_from_file("ray_geometry", "data/objects/cylinder.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.ray_geometry.Transform.value = \
            avango.gua.make_trans_mat(0.0,0.0,self.ray_length * -0.5) * \
            avango.gua.make_scale_mat(self.ray_thickness, self.ray_thickness, self.ray_length)
        self.ray_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))
        self.pointer_node.Children.value.append(self.ray_geometry)


        self.intersection_geometry = _loader.create_geometry_from_file("intersection_geometry", "data/objects/sphere.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.intersection_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))
        SCENEGRAPH.Root.value.Children.value.append(self.intersection_geometry)


        ### set initial states ###
        self.enable(False)



    ### functions ###
    def enable(self, BOOL): # extend respective base-class function
        ManipulationTechnique.enable(self, BOOL) # call base-class function

        if self.enable_flag == False:
            self.intersection_geometry.Tags.value = ["invisible"] # set intersection point invisible


    def update_ray_visualization(self, PICK_WORLD_POS = None, PICK_DISTANCE = 0.0):
        if PICK_WORLD_POS is None: # nothing hit
            # set ray to default length
            self.ray_geometry.Transform.value = \
                avango.gua.make_trans_mat(0.0,0.0,self.ray_length * -0.5) * \
                avango.gua.make_scale_mat(self.ray_thickness, self.ray_thickness, self.ray_length)
        
            self.intersection_geometry.Tags.value = ["invisible"] # set intersection point invisible

        else: # something hit
            # update ray length and intersection point
            self.ray_geometry.Transform.value = \
                avango.gua.make_trans_mat(0.0,0.0,PICK_DISTANCE * -0.5) * \
                avango.gua.make_scale_mat(self.ray_thickness, self.ray_thickness, PICK_DISTANCE)

            self.intersection_geometry.Tags.value = [] # set intersection point visible
            self.intersection_geometry.Transform.value = avango.gua.make_trans_mat(PICK_WORLD_POS) * avango.gua.make_scale_mat(self.intersection_point_size)


    ### callback functions ###
    def evaluate(self): # implement respective base-class function
        if self.enable_flag == False:
            return
    

        ## calc ray intersection
        ManipulationTechnique.update_intersection(self, PICK_MAT = self.pointer_node.WorldTransform.value, PICK_LENGTH = self.ray_length) # call base-class function

        ## update object selection
        ManipulationTechnique.selection(self) # call base-class function

        ## update visualizations
        if self.pick_result is None:
            self.update_ray_visualization() # apply default ray visualization
        else:
            _node = self.pick_result.Object.value # get intersected geometry node
    
            #_pick_pos = self.pick_result.Position.value # pick position in object coordinate system
            _pick_world_pos = self.pick_result.WorldPosition.value # pick position in world coordinate system
    
            _distance = self.pick_result.Distance.value * self.ray_length # pick distance in ray coordinate system
    
            #print(_node, _pick_pos, _pick_world_pos, _distance)
        
            self.update_ray_visualization(PICK_WORLD_POS = _pick_world_pos, PICK_DISTANCE = _distance)

        ## possibly perform object dragging
        ManipulationTechnique.dragging(self) # call base-class function

class DepthRay(ManipulationTechnique):

    ## constructor
    def __init__(self):
        self.super(DepthRay).__init__()


    def my_constructor(self,
        SCENEGRAPH = None,
        NAVIGATION_NODE = None,
        POINTER_INPUT = None,
        ):

        ManipulationTechnique.my_constructor(self, SCENEGRAPH, NAVIGATION_NODE, POINTER_INPUT) # call base-class constructor


        ### parameters ###
        self.ray_length = 2.5 # in meter
        self.ray_thickness = 0.01 # in meter
        self.depth_marker_size = 0.03

        
        ### resources ###
        _loader = avango.gua.nodes.TriMeshLoader()

        self.ray_geometry = _loader.create_geometry_from_file("ray_geometry", "data/objects/cylinder.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.ray_geometry.Transform.value = \
            avango.gua.make_trans_mat(0.0,0.0,self.ray_length * -0.5) * \
            avango.gua.make_scale_mat(self.ray_thickness, self.ray_thickness, self.ray_length)
        self.ray_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))
        self.pointer_node.Children.value.append(self.ray_geometry)

        self.marker_geometry = _loader.create_geometry_from_file("marker_geometry", "data/objects/sphere.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.marker_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(0.0, 0.0, 1.0, 1.0))
        self.marker_distance = 0.0
        self.marker_geometry.Transform.value = avango.gua.make_trans_mat(0.0, 0.0, self.marker_distance) * \
                                               avango.gua.make_scale_mat(self.depth_marker_size, self.depth_marker_size, self.depth_marker_size)
        self.pointer_node.Children.value.append(self.marker_geometry)

        ### set initial states ###
        self.highlighted_nodes = []
        self.enable(False)

    def enable(self, BOOL): # extend respective base-class function
        ManipulationTechnique.enable(self, BOOL) # call base-class function

    ### callback functions ###
    def evaluate(self): # implement respective base-class function
        if self.enable_flag == False:
            return

        q = self.pointer_node.Transform.value.get_rotate()
        angle   = -math.asin( 2 * q.x * q.y + 2 * q.z * q.w) * 180 / math.pi # equals to yaw
        if abs(angle) >= 15.0:
            angle *= 0.00005
            if (self.marker_distance + angle >= 0.0 and self.marker_distance + angle <= 1.0):
                self.marker_distance += angle

        self.marker_geometry.Transform.value = avango.gua.make_trans_mat(0.0, 0.0, self.marker_distance * -self.ray_length) * \
                                               avango.gua.make_scale_mat(self.depth_marker_size, self.depth_marker_size, self.depth_marker_size)

        ## calc ray intersection
        ManipulationTechnique.update_intersection(self, PICK_MAT = self.pointer_node.WorldTransform.value, PICK_LENGTH = self.ray_length) # call base-class function

        ## update object selection
        self.selection()

        ## drag object
        ManipulationTechnique.dragging(self)
    
    def selection(self):

        ## disable highlighting for objects which are not intersected
        if self.selected_node is not None:
            self.highlight(node=self.selected_node, color=avango.gua.Vec4(0.0, 1.0, 0.0, 0.3), color_override=False)
        for obj in self.highlighted_nodes:
            self.highlight(node=obj.Object.value.Parent.value, color=avango.gua.Vec4(0.0, 1.0, 0.0, 0.3), color_override=False)

        # add intersected objects to highlighted_nodes list
        self.highlighted_nodes.clear()
        if len(self.mf_pick_result.value) > 0:
            for obj in self.mf_pick_result.value:
                self.highlighted_nodes.append(obj)

        shortest_distance = self.ray_length
        for obj in self.highlighted_nodes:
            distance = abs(self.marker_distance - obj.Distance.value) 
            if distance < shortest_distance:
                shortest_distance = distance
                self.selected_node = obj.Object.value.Parent.value

        ## enable highlighting for objects in highlighted_nodes list
        for obj in self.highlighted_nodes:
            self.highlight(node=obj.Object.value.Parent.value, color=avango.gua.Vec4(0.0, 1.0, 0.0, 0.3))

        self.highlight(node=self.selected_node, color=avango.gua.Vec4(1.0, 0.0, 0.0, 0.5))


class GoGo(ManipulationTechnique):

    ## constructor
    def __init__(self):
        self.super(GoGo).__init__()


    def my_constructor(self,
        SCENEGRAPH = None,
        NAVIGATION_NODE = None,
        HEAD_NODE = None,
        POINTER_INPUT = None,
        ):

        ManipulationTechnique.my_constructor(self, SCENEGRAPH, NAVIGATION_NODE, POINTER_INPUT) # call base class constructor


        ### external references ###
        self.HEAD_NODE = HEAD_NODE
        

        ### parameters ###  
        self.intersection_point_size = 0.03 # in meter
        self.gogo_threshold = 0.35 # in meter

        self.head_to_body_offset = avango.gua.make_trans_mat(0.0, -0.45, 0.0) # we assume our body center is 45 cm lower than the head

        ### resources ###
        _loader = avango.gua.nodes.TriMeshLoader()

        self.hand_geometry = _loader.create_geometry_from_file("hand_geometry", "data/objects/hand.obj", avango.gua.LoaderFlags.DEFAULTS)
        #self.pointer_node.Tags.value = ["invisible"]
        self.hand_geometry.Transform.value = \
            avango.gua.make_scale_mat(3)
        self.hand_transform = avango.gua.nodes.TransformNode(Name = "hand_transform")
        self.hand_transform.Children.value = [self.hand_geometry]

        
        NAVIGATION_NODE.Children.value.append(self.hand_transform)

        ### set initial states ###


        self.enable(False)

    def enable(self, BOOL):
        self.enable_flag = BOOL
        
        if self.enable_flag == True:
            #self.pointer_node.Tags.value = [] # set tool visible
            self.hand_transform.Tags.value = [] # set hand visible
            self.SCENEGRAPH["/navigation/controller1_trans"].Tags.value = ["invisible"]

        else:
            self.stop_dragging() # possibly stop active dragging process
            
            self.hand_transform.Tags.value = ["invisible"] # set hand invisible
            self.SCENEGRAPH["/navigation/controller1_trans"].Tags.value = []

    ### callback functions ###
    def evaluate(self): # implement respective base-class function
        if self.enable_flag == False:
            return

        pointer_position = self.pointer_node.Transform.value.get_translate()
        body_center_position = (self.HEAD_NODE.Transform.value * self.head_to_body_offset).get_translate()
        body_center_to_pointer = pointer_position - body_center_position # vector in space
        distance_to_body_center = body_center_position.distance_to(pointer_position)

        self.hand_transform.Transform.value =  avango.gua.make_trans_mat(body_center_position) * \
                                               avango.gua.make_trans_mat(body_center_to_pointer * self.transfer(distance_to_body_center)) * \
                                               avango.gua.make_rot_mat(self.pointer_node.Transform.value.get_rotate())

         ## calc ray intersection
        self.update_intersection(PICK_MAT = self.hand_transform.WorldTransform.value, PICK_LENGTH = 0.20) # call base-class function

        ## update object selection
        self.selection() # call base-class function

        self.dragging()

    def transfer(self, distance):
        if distance <= self.gogo_threshold:
            return 1.0
        else:
            return (1.0 - self.gogo_threshold + distance)**3

    def start_dragging(self, NODE):
        self.dragged_node = NODE        
        self.dragging_offset_mat = avango.gua.make_inverse_mat(self.hand_transform.WorldTransform.value) * self.dragged_node.WorldTransform.value # object transformation in pointer coordinate system
    
    def dragging(self):
        if self.dragged_node is not None: # object to drag
            _new_mat = self.hand_transform.WorldTransform.value * self.dragging_offset_mat # new object position in world coodinates
            _new_mat = avango.gua.make_inverse_mat(self.dragged_node.Parent.value.WorldTransform.value) * _new_mat # transform new object matrix from global to local space
        
            self.dragged_node.Transform.value = _new_mat
        
            

class VirtualHand(ManipulationTechnique):

    ## constructor
    def __init__(self):
        self.super(ManipulationTechnique).__init__()


    def my_constructor(self,
        SCENEGRAPH = None,
        NAVIGATION_NODE = None,
        POINTER_INPUT = None
        ):

        ManipulationTechnique.my_constructor(self, SCENEGRAPH, NAVIGATION_NODE, POINTER_INPUT) # call base-class constructor


        ### parameters ###
        self.intersection_point_size = 0.03 # in meter

        self.min_vel = 0.01 / 60.0 # in meter/sec
        self.sc_vel = 0.15 / 60.0 # in meter/sec
        self.max_vel = 0.25 / 60.0 # in meter/sec

        self.last_frames = []
        self.last_frame_time = time.time()
        self.last_pointer_position = self.pointer_node.WorldTransform.value.get_translate()

        ### resources ###
        _loader = avango.gua.nodes.TriMeshLoader()

        self.hand_geometry = _loader.create_geometry_from_file("hand_geometry", "data/objects/hand.obj", avango.gua.LoaderFlags.DEFAULTS)
        #self.pointer_node.Tags.value = ["invisible"]
        self.hand_geometry.Transform.value = \
            avango.gua.make_scale_mat(3)
        self.hand_transform = avango.gua.nodes.TransformNode(Name = "hand_transform")
        self.hand_transform.Children.value = [self.hand_geometry]    

        NAVIGATION_NODE.Children.value.append(self.hand_transform)

        ### set initial states ###
        self.enable(False)

    def enable(self, BOOL):
        self.enable_flag = BOOL
        
        if self.enable_flag == True:
            #self.pointer_node.Tags.value = [] # set tool visible
            self.hand_transform.Tags.value = [] # set hand visible
            self.SCENEGRAPH["/navigation/controller1_trans"].Tags.value = ["invisible"]

            self.last_frame_time = time.time()
            self.last_pointer_position = self.pointer_node.WorldTransform.value.get_translate()

            # initialize hand position
            self.hand_transform.Transform.value = self.pointer_node.Transform.value
            self.last_hand_position = self.hand_transform.Transform.value.get_translate()

        else:
            self.stop_dragging() # possibly stop active dragging process
            
            self.hand_transform.Tags.value = ["invisible"] # set hand invisible
            self.SCENEGRAPH["/navigation/controller1_trans"].Tags.value = []


    ### callback functions ###
    def evaluate(self): # implement respective base-class function
        if self.enable_flag == False:
            return

        ## To-Do: implement Virtual Hand (with PRISM filter) technique here
        current_frame_time = time.time()
        delta_distance = self.pointer_node.Transform.value.get_translate() - self.last_pointer_position # Vector
        delta_time = current_frame_time - self.last_frame_time

        self.last_frames.append((current_frame_time, delta_time, delta_distance)) # add tuple
        self.last_frames = [element for element in self.last_frames if current_frame_time - element[0] <= 0.5] # only consider the last 500ms

        # calculate velocity vector
        velocity = avango.gua.Vec3(0, 0, 0)
        for frame in self.last_frames:
            frame_velocity = frame[2] / (frame[1] * 60)
            velocity += frame_velocity
        #velocity /= len(self.last_frames) # build the mean velocity by diving by the amount of last frames
        velocity /= len(self.last_frames) 

        D_hand = delta_distance
        print("D_hand:", D_hand)
        print("Kx:", self.K(velocity.x))
        print("Ky:", self.K(velocity.y))
        print("Kz:", self.K(velocity.z))
        D_object = avango.gua.Vec3(self.K(velocity.x) * D_hand.x, 
                                   self.K(velocity.y) * D_hand.y,
                                   self.K(velocity.z) * D_hand.z)

        print("D_object:", D_object)

        # update hand position
        self.hand_transform.Transform.value = avango.gua.make_trans_mat(D_object) *\
                                              avango.gua.make_trans_mat(self.last_hand_position) * \
                                              avango.gua.make_rot_mat(self.pointer_node.Transform.value.get_rotate())

        self.last_hand_position = self.hand_transform.Transform.value.get_translate()
        self.last_pointer_position = self.pointer_node.Transform.value.get_translate()
        self.last_frame_time = current_frame_time

    def K(self, S_hand):
        if abs(S_hand) >= self.sc_vel:
            return 1.0
        elif self.min_vel < abs(S_hand) and abs(S_hand) < self.sc_vel:
            return S_hand / self.sc_vel
        else:
            return 0.0