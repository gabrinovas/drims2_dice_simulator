import math
import os
import time
import random
import numpy as np

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Point, Vector3
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
from tf_transformations import quaternion_from_euler, quaternion_multiply
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle
from std_msgs.msg import Int16
from rcl_interfaces.srv import GetParameters

# Importar servicios personalizados
try:
    from drims2_msgs.srv import DiceIdentification, AttachObject
except ImportError:
    # Fallback para desarrollo/testing
    class DiceIdentification:
        class Request:
            pass
        class Response:
            def __init__(self):
                self.success = False
                self.face_number = 0
                self.pose = PoseStamped()
    
    class AttachObject:
        class Request:
            pass
        class Response:
            def __init__(self):
                self.success = False

try:
    import trimesh
    TRIMESH_AVAILABLE = True
except ImportError:
    TRIMESH_AVAILABLE = False
    print("⚠ trimesh no disponible. Usando cubo simple como fallback.")


class DiceSpawner(Node):
    def __init__(self):
        super().__init__('dice_spawner_node')

        # Declarar parámetros con valores por defecto
        self.declare_parameter("face_up", 0)
        self.declare_parameter("dice_size", 0.037)
        self.declare_parameter("position", [0.25, 0.0, 0.80])
        self.declare_parameter("use_sim_time", False)
        self.declare_parameter("dice_mesh", "simplify_Die-OBJ.obj")  # Nuevo parámetro para elegir mesh

        # Obtener parámetros
        face_param = self.get_parameter("face_up").value
        self.face = face_param if 1 <= face_param <= 6 else random.randint(1, 6)
        
        self.dice_size = self.get_parameter("dice_size").value
        pos_param = self.get_parameter("position").value
        self.position = Point(x=float(pos_param[0]), y=float(pos_param[1]), z=float(pos_param[2]))
        
        self.dice_mesh_file = self.get_parameter("dice_mesh").value

        self.dice_name = "dice"

        # Obtener path del mesh
        package_path = get_package_share_directory('drims2_dice_simulator')
        self.dice_mesh_path = os.path.join(package_path, 'urdf', self.dice_mesh_file)
        
        self.get_logger().info(f"Using mesh: {self.dice_mesh_path}")
        self.get_logger().info(f"Initializing dice with face {self.face} up, size {self.dice_size}m")

        # Node interno para callbacks separados
        self.internal_node = Node('dice_spawner_internal_node')
        self.internal_executor = MultiThreadedExecutor(num_threads=4)
        self.internal_executor.add_node(self.internal_node)

        # Callback groups
        self.service_callback_group = ReentrantCallbackGroup()
        self.get_scene_callback_group = ReentrantCallbackGroup()

        # Servicio de identificación de dado
        self.srv = self.create_service(
            DiceIdentification,
            '/dice_identification',
            self.get_dice_state_callback,
            callback_group=self.service_callback_group
        )

        # Publisher para la cara del dado
        self.dice_face_publisher_ = self.create_publisher(Int16, '/dice_face', 10)

        # Clientes de servicios
        self.apply_scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        while not self.apply_scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /apply_planning_scene service...')

        self.get_scene_client = self.internal_node.create_client(
            GetPlanningScene,
            '/get_planning_scene',
            callback_group=self.get_scene_callback_group
        )
        while not self.get_scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for /get_planning_scene service...")

        self.add_client = self.create_client(AttachObject, '/attach_object')
        while not self.add_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for /attach_object service...")

        # Normales de las caras del dado
        self.face_normals = {
            1: np.array([0, 1, 0]),   # Cara 1: frente (Y+)
            2: np.array([1, 0, 0]),   # Cara 2: derecha (X+)
            3: np.array([0, 0, -1]),  # Cara 3: abajo (Z-)
            4: np.array([0, 0, 1]),   # Cara 4: arriba (Z+)
            5: np.array([-1, 0, 0]),  # Cara 5: izquierda (X-)
            6: np.array([0, -1, 0]),  # Cara 6: atrás (Y-)
        }

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Obtener nombre del grupo y frame del mundo
        self.get_group_name()

        if self.group_name == "manipulator":
            self.get_logger().info("Using 'world' as world frame.")
            self.world = "world"
        else:
            self.get_logger().info("Using 'base_footprint' as world frame.")
            self.world = "base_footprint"

        # Publicar TFs estáticos y spawnear dado
        self.publish_all_static_transforms()
        time.sleep(1.0)  # Esperar a que las TFs se publiquen
        self.spawn_dice_with_mesh()
        self.dice_face_publisher_.publish(Int16(data=self.face))
        
        self.get_logger().info("Dice spawner initialized successfully!")

    def get_group_name(self):
        """Obtener el nombre del grupo de MoveIt del servidor de movimiento"""
        self.group_name = None
        param_client = self.internal_node.create_client(GetParameters, '/motion_server_node/get_parameters')
        
        try:
            while not param_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info("Waiting for /motion_server_node/get_parameters service...")

            param_request = GetParameters.Request()
            param_request.names = ['move_group_name']

            future = param_client.call_async(param_request)
            self.internal_executor.spin_until_future_complete(future, timeout_sec=5.0)

            if future.done() and future.result() is not None:
                values = future.result().values
                if values and values[0].string_value:
                    self.group_name = values[0].string_value
                    self.get_logger().info(f"Retrieved group_name: {self.group_name}")
                else:
                    self.get_logger().warn("Parameter 'group_name' is empty or not set. Using 'manipulator' as default.")
                    self.group_name = "manipulator"
            else:
                self.get_logger().warn("Failed to get 'group_name' from /motion_server_node. Using 'manipulator' as default.")
                self.group_name = "manipulator"
                
        except Exception as e:
            self.get_logger().warn(f"Exception while getting group_name: {e}. Using 'manipulator' as default.")
            self.group_name = "manipulator"

    def publish_all_static_transforms(self):
        """Publicar todas las transformadas estáticas iniciales del dado"""
        transforms = []

        # TF base (posición del dado)
        tf_base = TransformStamped()
        tf_base.header.stamp = self.get_clock().now().to_msg()
        tf_base.header.frame_id = self.world
        tf_base.child_frame_id = "dice_base_tf"
        tf_base.transform.translation = Vector3(
            x=self.position.x,
            y=self.position.y,
            z=self.position.z
        )
        tf_base.transform.rotation.x = 0.0
        tf_base.transform.rotation.y = 0.0
        tf_base.transform.rotation.z = 0.0
        tf_base.transform.rotation.w = 1.0
        transforms.append(tf_base)

        # TF rotated (orientación para la cara deseada)
        q = self.get_orientation_for_face(self.face)
        tf_rot = TransformStamped()
        tf_rot.header.stamp = self.get_clock().now().to_msg()
        tf_rot.header.frame_id = "dice_base_tf"
        tf_rot.child_frame_id = "dice_rotated_tf"
        tf_rot.transform.translation.x = 0.0
        tf_rot.transform.translation.y = 0.0
        tf_rot.transform.translation.z = 0.0
        tf_rot.transform.rotation.x = q[0]
        tf_rot.transform.rotation.y = q[1]
        tf_rot.transform.rotation.z = q[2]
        tf_rot.transform.rotation.w = q[3]
        transforms.append(tf_rot)

        # TFs para cada cara (centros de las caras)
        for face_id, normal in self.face_normals.items():
            offset = (self.dice_size / 2.0) * normal
            q_face = self.get_quaternion_from_normal(normal)
            tf_face = TransformStamped()
            tf_face.header.stamp = self.get_clock().now().to_msg()
            tf_face.header.frame_id = "dice_rotated_tf"
            tf_face.child_frame_id = f"face{face_id}_tf"
            tf_face.transform.translation.x = float(offset[0])
            tf_face.transform.translation.y = float(offset[1])
            tf_face.transform.translation.z = float(offset[2])
            tf_face.transform.rotation.x = q_face[0]
            tf_face.transform.rotation.y = q_face[1]
            tf_face.transform.rotation.z = q_face[2]
            tf_face.transform.rotation.w = q_face[3]
            transforms.append(tf_face)

        # TF del dado (centrado en la cara superior actual)
        tf_dice = TransformStamped()
        tf_dice.header.stamp = self.get_clock().now().to_msg()
        tf_dice.header.frame_id = f"face{self.face}_tf"
        tf_dice.child_frame_id = "dice_tf"
        tf_dice.transform.translation.x = 0.0
        tf_dice.transform.translation.y = 0.0
        tf_dice.transform.translation.z = 0.0
        tf_dice.transform.rotation.x = 0.0
        tf_dice.transform.rotation.y = 0.0
        tf_dice.transform.rotation.z = 0.0
        tf_dice.transform.rotation.w = 1.0
        transforms.append(tf_dice)

        self.static_tf_broadcaster.sendTransform(transforms)
        self.get_logger().info(f"Published {len(transforms)} static transforms for dice")

    def update_dice_tf_from_scene(self):
        """Actualizar las TFs del dado basado en la escena de planificación actual"""
        try:
            request = GetPlanningScene.Request()
            request.components.components = (
                GetPlanningScene.Request().components.WORLD_OBJECT_NAMES |
                GetPlanningScene.Request().components.WORLD_OBJECT_GEOMETRY |
                GetPlanningScene.Request().components.WORLD_OBJECT_POSES |
                GetPlanningScene.Request().components.ROBOT_STATE_ATTACHED_OBJECTS
            )

            future = self.get_scene_client.call_async(request)
            self.internal_executor.spin_until_future_complete(future, timeout_sec=5.0)

            if not future.done():
                self.get_logger().warn("Timeout while waiting for planning scene.")
                return False

            result = future.result()

            # Buscar dado en objetos de colisión del mundo
            for obj in result.scene.world.collision_objects:
                if obj.id == self.dice_name:
                    self.publish_updated_dice_rotated_tf(obj.pose, obj.header.frame_id)
                    self.get_logger().info("Found dice in world collision objects")
                    return True

            # Buscar dado en objetos adjuntos al robot
            for attached_obj in result.scene.robot_state.attached_collision_objects:
                if attached_obj.object.id == self.dice_name:
                    self.publish_updated_dice_rotated_tf(attached_obj.object.pose, attached_obj.object.header.frame_id)
                    self.get_logger().info("Found dice attached to robot")
                    return True

            self.get_logger().warn(f"Dice object '{self.dice_name}' not found in planning scene.")
            return False

        except Exception as e:
            self.get_logger().error(f'Error in update_dice_tf_from_scene: {str(e)}')
            return False

    def publish_updated_dice_rotated_tf(self, pose: Pose, parent_frame: str):
        """Publicar TFs actualizadas del dado"""
        transforms = []

        # TF rotated actualizada
        tf_rot = TransformStamped()
        tf_rot.header.stamp = self.get_clock().now().to_msg()
        tf_rot.header.frame_id = parent_frame
        tf_rot.child_frame_id = "dice_rotated_tf"
        tf_rot.transform.translation = Vector3(
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z
        )
        tf_rot.transform.rotation = pose.orientation
        transforms.append(tf_rot)

        # TFs de las caras (relativas a la orientación actual)
        for face_id, normal in self.face_normals.items():
            offset = (self.dice_size / 2.0) * normal
            q = self.get_quaternion_from_normal(normal)
            tf_face = TransformStamped()
            tf_face.header.stamp = self.get_clock().now().to_msg()
            tf_face.header.frame_id = "dice_rotated_tf"
            tf_face.child_frame_id = f"face{face_id}_tf"
            tf_face.transform.translation.x = float(offset[0])
            tf_face.transform.translation.y = float(offset[1])
            tf_face.transform.translation.z = float(offset[2])
            tf_face.transform.rotation.x = q[0]
            tf_face.transform.rotation.y = q[1]
            tf_face.transform.rotation.z = q[2]
            tf_face.transform.rotation.w = q[3]
            transforms.append(tf_face)

        self.static_tf_broadcaster.sendTransform(transforms)
        self.get_logger().info(f"Updated {len(transforms)} transforms for moved dice")

    def spawn_dice_with_mesh(self):
        """Spawnear el dado en la escena de planificación usando mesh OBJ"""
        # Crear pose del dado
        pose = PoseStamped()
        pose.header.frame_id = "dice_rotated_tf"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        # Crear mesh - intentar OBJ primero, luego fallback a cubo simple
        if TRIMESH_AVAILABLE and os.path.exists(self.dice_mesh_path):
            mesh_msg = self.load_obj_mesh()
        else:
            self.get_logger().warn(f"OBJ mesh not available, using simple cube. trimesh: {TRIMESH_AVAILABLE}, file exists: {os.path.exists(self.dice_mesh_path)}")
            mesh_msg = self.create_simple_cube_mesh()

        # Crear objeto de colisión
        obj = CollisionObject()
        obj.id = self.dice_name
        obj.header = pose.header
        obj.meshes = [mesh_msg]
        obj.mesh_poses = [pose.pose]
        obj.operation = CollisionObject.ADD

        # Aplicar escena
        scene = PlanningScene()
        scene.world.collision_objects = [obj]
        scene.is_diff = True

        req = ApplyPlanningScene.Request(scene=scene)
        future = self.apply_scene_client.call_async(req)
        future.add_done_callback(self.spawn_dice_result)

        mesh_type = "OBJ" if TRIMESH_AVAILABLE and os.path.exists(self.dice_mesh_path) else "SIMPLE CUBE"
        self.get_logger().info(f"Spawned {mesh_type} dice with:\n" +
                              f" - Face {self.face} up\n" +
                              f" - Position [{self.position.x:.3f}, {self.position.y:.3f}, {self.position.z:.3f}]\n" +
                              f" - Size {self.dice_size:.3f}m\n" +
                              f" - Mesh: {self.dice_mesh_file}")

    def load_obj_mesh(self):
        """Cargar mesh desde archivo OBJ usando trimesh"""
        try:
            mesh = trimesh.load(self.dice_mesh_path, force='mesh')
            mesh_msg = Mesh()
            
            # Escalar el mesh al tamaño del dado
            scale_factor = self.dice_size / max(mesh.extents)  # Normalizar al tamaño máximo
            scaled_vertices = mesh.vertices * scale_factor
            
            # Triángulos
            for tri in mesh.faces:
                triangle = MeshTriangle()
                triangle.vertex_indices = tri.tolist()
                mesh_msg.triangles.append(triangle)
            
            # Vértices escalados
            for v in scaled_vertices:
                point = Point()
                point.x, point.y, point.z = v
                mesh_msg.vertices.append(point)
                
            self.get_logger().info(f"Loaded OBJ mesh: {len(mesh_msg.vertices)} vertices, {len(mesh_msg.triangles)} triangles")
            return mesh_msg
            
        except Exception as e:
            self.get_logger().error(f"Error loading OBJ mesh: {e}")
            return self.create_simple_cube_mesh()

    def create_simple_cube_mesh(self):
        """Crear un mesh de cubo simple como fallback"""
        mesh_msg = Mesh()
        
        # Definir vértices del cubo (escalados por dice_size/2)
        half_size = self.dice_size / 2.0
        vertices = [
            [-half_size, -half_size, -half_size],
            [ half_size, -half_size, -half_size],
            [ half_size,  half_size, -half_size],
            [-half_size,  half_size, -half_size],
            [-half_size, -half_size,  half_size],
            [ half_size, -half_size,  half_size],
            [ half_size,  half_size,  half_size],
            [-half_size,  half_size,  half_size],
        ]
        
        # Definir triángulos (caras del cubo)
        triangles = [
            [0, 1, 2], [2, 3, 0],  # bottom
            [4, 5, 6], [6, 7, 4],  # top
            [0, 1, 5], [5, 4, 0],  # front
            [2, 3, 7], [7, 6, 2],  # back
            [0, 3, 7], [7, 4, 0],  # left
            [1, 2, 6], [6, 5, 1],  # right
        ]
        
        # Añadir vértices
        for v in vertices:
            point = Point()
            point.x, point.y, point.z = v
            mesh_msg.vertices.append(point)
        
        # Añadir triángulos
        for tri in triangles:
            triangle = MeshTriangle()
            triangle.vertex_indices = tri
            mesh_msg.triangles.append(triangle)
            
        self.get_logger().info(f"Created simple cube mesh with {len(mesh_msg.vertices)} vertices and {len(mesh_msg.triangles)} triangles")
        return mesh_msg

    def spawn_dice_result(self, future):
        """Callback del resultado del spawn del dado"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("✓ Dice spawned successfully in planning scene")
            else:
                self.get_logger().warn("⚠ Dice spawn completed but success=False")
        except Exception as e:
            self.get_logger().error(f'✗ Error while spawning dice: {str(e)}')

    def get_dice_state_callback(self, request, response):
        """Servicio callback para identificar el estado del dado"""
        self.get_logger().info("🎲 Received dice identification request")
        try:
            success = self.update_dice_tf_from_scene()
            if not success:
                self.get_logger().warn("Failed to update dice transform from planning scene.")
                response.success = False
                return response

            time.sleep(0.5)  # Esperar a que se actualicen las TFs

            now = rclpy.time.Time()
            z_world = np.array([0, 0, 1])
            best_face = None
            best_dot = -1.0
            best_tf = None

            # Encontrar la cara que más apunta hacia arriba
            for face_id in range(1, 7):
                try:
                    tf = self.tf_buffer.lookup_transform(self.world, f'face{face_id}_tf', now)
                    q = tf.transform.rotation
                    q_np = np.array([q.x, q.y, q.z, q.w])
                    z_local = np.array([0, 0, 1])
                    z_world_face = self.rotate_vector(z_local, q_np)
                    dot = np.dot(z_world_face, z_world)
                    
                    self.get_logger().debug(f"Face {face_id}: dot product with world up = {dot:.3f}")
                    
                    if dot > best_dot:
                        best_dot = dot
                        best_face = face_id
                        best_tf = tf
                except Exception as e:
                    self.get_logger().warn(f"Could not transform face{face_id}_tf: {e}")
                    continue

            if best_face is None:
                self.get_logger().error("Could not determine dice face - no valid transforms found")
                response.success = False
                return response

            self.face = best_face
            self.dice_face_publisher_.publish(Int16(data=best_face))

            # Crear respuesta con pose
            pose = PoseStamped()
            pose.header = best_tf.header
            pose.pose.position = Point(
                x=best_tf.transform.translation.x,
                y=best_tf.transform.translation.y,
                z=best_tf.transform.translation.z
            )
            pose.pose.orientation = best_tf.transform.rotation

            self.get_logger().info(f"🎯 Detected face up: {best_face} (dot: {best_dot:.3f})")
            self.get_logger().info(f"📍 Position: x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}")

            # Actualizar TF del dado
            dice_tf = TransformStamped()
            dice_tf.header.stamp = self.get_clock().now().to_msg()
            dice_tf.header.frame_id = f"face{self.face}_tf"
            dice_tf.child_frame_id = "dice_tf"
            dice_tf.transform.translation.x = 0.0
            dice_tf.transform.translation.y = 0.0
            dice_tf.transform.translation.z = 0.0
            dice_tf.transform.rotation.x = 0.0
            dice_tf.transform.rotation.y = 0.0
            dice_tf.transform.rotation.z = 0.0
            dice_tf.transform.rotation.w = 1.0
            self.static_tf_broadcaster.sendTransform([dice_tf])

            response.pose = pose
            response.face_number = best_face
            response.success = True
            
            self.get_logger().info("✅ Dice identification completed successfully")
            return response

        except Exception as e:
            self.get_logger().error(f"❌ get_dice_state_callback error: {e}")
            response.success = False
            return response

    def get_orientation_for_face(self, face):
        """Obtener orientación quaternion para una cara específica hacia arriba"""
        face_to_rpy = {
            1: (math.pi / 2, 0, 0),
            2: (0, -math.pi / 2, 0),
            3: (0, math.pi, 0),
            4: (0, 0, 0),
            5: (0, math.pi / 2, 0),
            6: (-math.pi / 2, 0, 0),
        }
        rpy = face_to_rpy.get(face, (0, 0, 0))
        return quaternion_from_euler(*rpy)

    def get_quaternion_from_normal(self, normal):
        """Obtener quaternion que alinea el eje Z con la normal dada"""
        z_axis = np.array([0, 0, 1])
        v = np.cross(z_axis, normal)
        c = np.dot(z_axis, normal)
        
        if np.linalg.norm(v) < 1e-6:
            return (0.0, 0.0, 0.0, 1.0) if c > 0 else quaternion_from_euler(math.pi, 0, 0)
        
        s = math.sqrt((1 + c) * 2)
        vx, vy, vz = v / np.linalg.norm(v)
        return (
            vx * math.sin(math.acos(c) / 2),
            vy * math.sin(math.acos(c) / 2),
            vz * math.sin(math.acos(c) / 2),
            math.cos(math.acos(c) / 2),
        )

    def rotate_vector(self, v, q):
        """Rotar un vector por un quaternion"""
        v_q = (v[0], v[1], v[2], 0.0)
        q_conj = (-q[0], -q[1], -q[2], q[3])
        result = quaternion_multiply(quaternion_multiply(q, v_q), q_conj)
        return result[:3]

    def destroy_node(self):
        """Cleanup al destruir el nodo"""
        self.internal_executor.shutdown()
        self.internal_node.destroy_node()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DiceSpawner()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        
        node.get_logger().info("🎲 Dice Spawner started. Press Ctrl+C to exit.")
        executor.spin()
        
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Keyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"❌ Error in main: {e}")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print("🎲 Dice Spawner shutdown complete.")


if __name__ == '__main__':
    main()