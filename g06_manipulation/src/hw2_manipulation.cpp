#include <g06_manipulation/ROSMoveit.h>
#include <g06_perception/ROSUtils.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <shape_msgs/SolidPrimitive.h>
#include <geometric_shapes/shape_operations.h>

#include <geometry_msgs/Vector3.h>

// Dichiarazione variabili globali per il nodo
bool simulated = true;


/**
 * Imposta la posizione iniziale del manipolatore
 * @return  la posizione iniziale del manipolatore
 */
geometry_msgs::Pose getInitialPose() {
	geometry_msgs::Pose initial_pose;
	initial_pose.orientation.x = -0.698151;
	initial_pose.orientation.y = -0.00102259;
	initial_pose.orientation.z = 0.715943;
	initial_pose.orientation.w = 0.000674622;
	initial_pose.position.x = 0.0182307;
	initial_pose.position.y = -0.0106809;
	initial_pose.position.z = 1.55191;
	return initial_pose;
}//getInitialPose


/**
 * Punto di inizio dell'esecuzione
 * @param  argc  numero di parametri
 * @param  argv  stringhe rappresentanti i parametri
 * @return  il codice di stato alla fine dell'esecuzione
 */
int main(int argc, char **argv) {

	// Inizializzazione ROS
    ros::init(argc, argv, "hw2");
	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::NodeHandle nh;	

	if(nh.hasParam("sim")) {
		nh.getParam("sim", simulated);
	}//if
	std::cout << "STARTS ";
	if(!simulated)
		std::cout << "NOT ";
	std::cout << "SIMULATED" << std::endl;

	// Utilità
	ros_hw_utils::ROSUtils utils(nh, simulated);
	ros_hw_utils::ROSMoveit moveitUtils(nh, "manipulator", simulated);

	do {
		moveitUtils.removeCollisionObjects();
		geometry_msgs::Pose basketPose;
		int numDetected;
		std::vector<apriltags_ros::AprilTagDetection> vectDet;
		int numRequested;
		std::vector<apriltags_ros::AprilTagDetection> vectReq;

		// Comunicazione del proprio stato "Pronto" al server
		ros::ServiceClient clientManipulationReady = nh.serviceClient<g06_fsm::ManipulationReady>("/ManipulationReady");
		g06_fsm::ManipulationReady serviceMR;
		serviceMR.request.iamready = 1;
		if(clientManipulationReady.call(serviceMR)) {
			ROS_INFO_STREAM("Call effettuata! ManipulationReady");
			if(serviceMR.response.received == 1) {
				ROS_INFO_STREAM("Tutto ok! ManipulationReady");
				numDetected = serviceMR.response.numFound;
				vectDet = serviceMR.response.detections_all.detections;
				numRequested = serviceMR.response.numRequested;
				vectReq = serviceMR.response.detections_tograb.detections;
				basketPose = serviceMR.response.robotPose.pose;
			} else {
				ROS_INFO_STREAM("Non ricevuto! ManipulationReady");
				break;
			}//if else
		} else {
			ROS_ERROR_STREAM("Errore! ManipulationReady");
		}//if else

		geometry_msgs::Pose object_pose;
		moveit_msgs::CollisionObject co;
		shapes::Mesh* m;
		shape_msgs::Mesh mesh;
		shapes::ShapeMsg mesh_msg;

		std::vector<std::string> collId;

		// Creazione degli oggetti di collisione tramite mesh in base agli oggetti presenti sul tavolo
		for(int i = 0; i < vectDet.size(); i++) {

			object_pose = utils.doTransform(vectDet[i].pose.pose);
			object_pose.position.z += 0.3;
			
			switch(utils.getPrimitiveShape((vectDet[i].id))) {
				case ros_hw_utils::ROSUtils::SHAPE_CUBE:
					// Creazione di un cubo tramite mesh
					m = shapes::createMeshFromResource("package://challenge_arena/meshes/octagon_big.dae");
					shapes::constructMsgFromShape(m, mesh_msg);
					mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

					co.meshes.resize(1);
					co.mesh_poses.resize(1);
					co.meshes[0] = mesh;
					co.id = utils.getModelNameById(vectDet[i].id);
					co.header.frame_id = "world";
					co.mesh_poses[0].position.x = object_pose.position.x;
					co.mesh_poses[0].position.y = object_pose.position.y;
					co.mesh_poses[0].position.z = object_pose.position.z - 0.35;
					co.mesh_poses[0].orientation.w = object_pose.orientation.w; 
					co.mesh_poses[0].orientation.x = object_pose.orientation.x;
					co.mesh_poses[0].orientation.y = object_pose.orientation.y;
					co.mesh_poses[0].orientation.z = object_pose.orientation.z;
					co.meshes.push_back(mesh);
					co.mesh_poses.push_back(co.mesh_poses[0]);
					co.operation = co.ADD;
					break;

				case ros_hw_utils::ROSUtils::SHAPE_CYLINDER:
					// Creazione di un cilindro tramite mesh
					m = shapes::createMeshFromResource("package://challenge_arena/meshes/hexagon.dae");
					shapes::constructMsgFromShape(m, mesh_msg);
					mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

					co.meshes.resize(1);
					co.mesh_poses.resize(1);
					co.meshes[0] = mesh;
					co.id = utils.getModelNameById(vectDet[i].id);
					co.header.frame_id = "world";
					co.mesh_poses[0].position.x = object_pose.position.x;
					co.mesh_poses[0].position.y = object_pose.position.y;
					co.mesh_poses[0].position.z = object_pose.position.z - 0.4;
					co.mesh_poses[0].orientation.w = object_pose.orientation.w; 
					co.mesh_poses[0].orientation.x = object_pose.orientation.x;
					co.mesh_poses[0].orientation.y = object_pose.orientation.y;
					co.mesh_poses[0].orientation.z = object_pose.orientation.z;
					co.meshes.push_back(mesh);
					co.mesh_poses.push_back(co.mesh_poses[0]);
					co.operation = co.ADD;
					break;

				case ros_hw_utils::ROSUtils::SHAPE_TRIANGLE:
					// Creazione di un triangolo tramite mesh
					m = shapes::createMeshFromResource("package://challenge_arena/meshes/octagon_big.dae");
					shapes::constructMsgFromShape(m, mesh_msg);
					mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

					co.meshes.resize(1);
					co.mesh_poses.resize(1);
					co.meshes[0] = mesh;
					co.id = utils.getModelNameById(vectDet[i].id);
					co.header.frame_id = "world";
					co.mesh_poses[0].position.x = object_pose.position.x;
					co.mesh_poses[0].position.y = object_pose.position.y;
					co.mesh_poses[0].position.z = object_pose.position.z - 0.35;
					co.mesh_poses[0].orientation.w = object_pose.orientation.w; 
					co.mesh_poses[0].orientation.x = object_pose.orientation.x;
					co.mesh_poses[0].orientation.y = object_pose.orientation.y;
					co.mesh_poses[0].orientation.z = object_pose.orientation.z;
					co.meshes.push_back(mesh);
					co.mesh_poses.push_back(co.mesh_poses[0]);
					co.operation = co.ADD;
					break;

				default:
					ROS_INFO("Not valid");
			}//switch

			collId.push_back(moveitUtils.addCollisionObject(co));
		}//for

        moveitUtils.applyCollisionObjectsToScene();

		// Impostiamo una posizione iniziale indicandola in giunti
		std::vector<double> joint_initial_positions(6);
		joint_initial_positions[0] = 1.91;
		joint_initial_positions[1] = -1.18;
		joint_initial_positions[2] = -1.82;
		joint_initial_positions[3] = -1.69;
		joint_initial_positions[4] = 1.56;
		joint_initial_positions[5] = 1.80;
		moveitUtils.setPoseWithJoints(joint_initial_positions, true);

		// Creiamo ora dei muri in modo da evitare che il manipolatore faccia giri inutilmente complessi
		geometry_msgs::Pose wall1_pose;
		wall1_pose.orientation.w = 1.0;
		wall1_pose.position.x = 0.90;
		wall1_pose.position.y = 0;
		wall1_pose.position.z = 1.1;
		moveitUtils.createWall("wall1", wall1_pose, 0.1, 1, 2.2);

		geometry_msgs::Pose wall2_pose;
		wall2_pose.orientation.w = 1.0;
		wall2_pose.position.x = 0;
		wall2_pose.position.y = 0;
        wall2_pose.position.z = 2.0;
		moveitUtils.createWall("wall2", wall2_pose, 1.8, 1, 0.1);

        // Oggetto che evita giri complessi al manipolatore, evita il movimento su se stesso di tutto il manipolatore
		geometry_msgs::Pose small_object_pose;
		small_object_pose.orientation.w = 1.0;
		small_object_pose.position.x = 0.55;
		small_object_pose.position.y = -0.2;
		small_object_pose.position.z = 1;
		moveitUtils.createWall("small_object", small_object_pose, 0.10, 0.08, 0.2);

		if(!simulated) {
			geometry_msgs::Pose wall3_pose;
			wall3_pose.orientation.w = 1.0;
			wall3_pose.position.x = 0;
			wall3_pose.position.y = -0.5;
			wall3_pose.position.z = 1.1;
			moveitUtils.createWall("wall3", wall3_pose, 1.8, 0.1, 2.2);
		}//if		

		moveitUtils.applyCollisionObjectsToScene();

		geometry_msgs::Pose target_pose_tr;

		int triangle = 0;
		int cyl = 0;
		int cube = 0;

		// Prendiamo gli oggetti richiesti
		for(int i = 0; i < vectReq.size(); i++) {
			// Usiamo dei waypoint per spostare il manipolatore verso la posizione finale
			std::vector<geometry_msgs::Pose> waypoints;
			waypoints.push_back(moveitUtils.getEndEffectorPose());

			// Impostiamo una posizione rialzata rispetto all'oggetto
			target_pose_tr = utils.doTransform(vectReq[i].pose.pose);
			target_pose_tr.position.z += 0.3;

			//Utilizziamo RPY per muovere l'end effector nella posizione migliore per prendere l'oggetto
			tf::Quaternion q(target_pose_tr.orientation.x, target_pose_tr.orientation.y, target_pose_tr.orientation.z, target_pose_tr.orientation.w);
			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);

			std::vector<double> rpy_ee = moveitUtils.getEndEffectorRPY();

			if(vectReq[i].id == 6 || vectReq[i].id == 7 || vectReq[i].id == 8) {
				// Triangoli verdi
				rpy_ee[2] = std::fmod(yaw - M_PI - M_PI/12, M_PI);
			} else if(vectReq[i].id == 13 || vectReq[i].id == 14 || vectReq[i].id == 15) {
				// Triangoli rossi
				rpy_ee[2] = std::fmod(yaw - M_PI - M_PI/12, M_PI);
			} else {
				rpy_ee[2] = 2.41922;
			}//if else

			q = tf::createQuaternionFromRPY(rpy_ee[0], rpy_ee[1], rpy_ee[2]);

			target_pose_tr.orientation.x = q[0];
			target_pose_tr.orientation.y = q[1];
			target_pose_tr.orientation.z = q[2];
			target_pose_tr.orientation.w = q[3];

			if(utils.getPrimitiveShape((vectReq[i].id)) == ros_hw_utils::ROSUtils::SHAPE_CYLINDER){
				target_pose_tr.orientation.x = 0.707427;
				target_pose_tr.orientation.y = -0.00221944;
				target_pose_tr.orientation.z = -0.706485;
				target_pose_tr.orientation.w = -0.0205306;
			}//if

			waypoints.push_back(target_pose_tr);

			// Abbassiamo la posizione precedente in base al tipo di oggetto da prendere
			if (utils.getPrimitiveShape((vectReq[i].id)) == ros_hw_utils::ROSUtils::SHAPE_TRIANGLE) {
				target_pose_tr.position.x += 0.04;
				target_pose_tr.position.y -= 0.03;
				target_pose_tr.position.z -= 0.115;
			} else {
				target_pose_tr.position.y -= 0.015;
				target_pose_tr.position.z -= 0.15;
			}//if else
			
			waypoints.push_back(target_pose_tr);

			moveitUtils.setPoseWithCartesianPath(waypoints, true);

			// Eseguiamo l'attach l'oggetto al gripper
			moveitUtils.attachObject(utils.getModelNameById(vectReq[i].id));
			std::cout << "ATTACHED: " << moveitUtils.getAttachedObject().id << std::endl;

			// Chiudiamo il gripper
			moveitUtils.closeGripper();

            std::vector<geometry_msgs::Pose> waypoints2;
            waypoints2.push_back(moveitUtils.getEndEffectorPose());
			target_pose_tr.position.z += 0.27;
            waypoints2.push_back(target_pose_tr);

            waypoints2.push_back(utils.createPose(-0.0337, 0.6860, 1.453, 0.6121, -0.3586, -0.606, -0.359));

			if(simulated) {
				if(utils.getPrimitiveShape(vectReq[i].id) == utils.SHAPE_CYLINDER) {
					moveitUtils.setPose(utils.createPose(0.305997, 1.06683, 1.11475, 0.919761, 0.000371658, -0.392478, -0.0011659), true);
				} else {
					moveitUtils.setPoseWithCartesianPath(waypoints, true);
					std::cout << "basketPose x: " << basketPose.position.x << std::endl; 
					std::cout << "basketPose y: " << basketPose.position.y << std::endl; 
					std::cout << "basketPose z: " << basketPose.position.z << std::endl; 
					moveitUtils.setPose(utils.createPose(basketPose.position.x - 0.3, basketPose.position.y - 0.6, basketPose.position.z + 1.5, -0.698151, -0.00102259, 0.715943, 0.000674622), true);
				}//if else
			}//if

			if(!simulated) {
				if(utils.getPrimitiveShape(vectReq[i].id) == utils.SHAPE_TRIANGLE) {
					if (triangle == 0) {
						// pose primo triangolo
                        waypoints2.push_back(utils.createPose(0.5220, 1.07, 1.0, 0.541158, -0.4817, -0.489, -0.4856));
						triangle++;
					} else {
						// pose secondo triangolo
                        waypoints2.push_back(utils.createPose(0.6434, 1.07, 1.0, 0.541158, -0.4817, -0.489, -0.4856));
						triangle++;
					}//if else
				} else if (utils.getPrimitiveShape(vectReq[i].id) == utils.SHAPE_CYLINDER) {
					if (cyl == 0) {
						// pose primo cilindro
						waypoints2.push_back(utils.createPose(0.53, 1.15, 1.05, 0.911, -0.0549, -0.4059, 0.0437));
						cyl++;
					} else {
						// pose secondo cilindro
						waypoints2.push_back(utils.createPose(0.53, 1.08, 1.05, 0.911, -0.0549, -0.4059, 0.0437));
						cyl++;
					}//if else
				} else if (utils.getPrimitiveShape(vectReq[i].id) == utils.SHAPE_CUBE) {
					if (cube == 0) {
						// pose primo cubo
						waypoints2.push_back(utils.createPose(0.5020, 1.13, 1.04334, 0.541158, -0.4817, -0.489, -0.4856));
						cube++;
					} else if (cube == 1) {
						// pose secondo cubo
						waypoints2.push_back(utils.createPose(0.7029, 1.09922, 1.04334, 0.541158, -0.4817, -0.489, -0.4856));
						cube++;
					} else {
						// pose terzo cubo
						waypoints2.push_back(utils.createPose(0.5370, 1.03, 1.04334, 0.541158, -0.4817, -0.489, -0.4856));
						cube++;
					}//if else
				} else {
					// Non deve mai arrivare a questa situazione, gli oggetti non noti vengono scartati dal nodo Perception
					ROS_INFO_STREAM("Il tag non e' associato a nessun oggetto noto");
				}//if else
			}//if

			moveitUtils.setPoseWithCartesianPath(waypoints2, true);

			// Eseguiamo il detach dell'oggetto ed apriamo il gripper
			moveitUtils.detachObject();
			moveitUtils.openGripper();

			// Riportiamo il manipolatore nella posizione iniziale
            std::vector<double> joint_initial_positions(6);
            joint_initial_positions[0] = 1.91;
            joint_initial_positions[1] = -1.18;
            joint_initial_positions[2] = -1.82;
            joint_initial_positions[3] = -1.69;
            joint_initial_positions[4] = 1.56;
            joint_initial_positions[5] = 1.80;
            moveitUtils.setPoseWithJoints(joint_initial_positions, true);
		}//for

		// Comunicazione di carico avvenuto al server
		ros::ServiceClient clientManipulationToFSM = nh.serviceClient<g06_fsm::ManipulationToFSM>("/ManipulationToFSM");
		g06_fsm::ManipulationToFSM serviceM2FSM;
		serviceM2FSM.request.done = 1;
		serviceM2FSM.request.numLoaded = vectReq.size();
		if(clientManipulationToFSM.call(serviceM2FSM)) {
			if(serviceM2FSM.response.ack == 1) {
				ROS_INFO_STREAM("Caricamento effettuato! ManipulationToFSM");
				if(serviceM2FSM.response.numRemaining == 0) {
					// Sono stati caricati tutti gli oggetti richiesti, fine delle operazioni
					ROS_INFO_STREAM("Caricamenti terminati! ManipulationToFSM");
					break;
				}//if
			} else {
				ROS_ERROR_STREAM("Errore! ManipulationToFSM");
				break;
			}//if else
		} else {
			ROS_ERROR_STREAM("Errore! ManipulationToFSM");
		}//if else

	} while(true);
	
    return 0;
}//main