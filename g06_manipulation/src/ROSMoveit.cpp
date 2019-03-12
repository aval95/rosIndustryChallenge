#include <g06_manipulation/ROSMoveit.h>

/**
 * Costruttore, inizializza le variabili d'istanza
 * @param  nodehandle  l'oggetto NodeHandle relativo al nodo corrente
 * @param  simulated  indica se sis sta usando la simulazione oppure no
 */
ros_hw_utils::ROSMoveit::ROSMoveit(ros::NodeHandle nodehandle, std::string move_group_name, bool simulated) : m_nh(nodehandle), m_move_group_name(move_group_name), m_move_group("manipulator"), m_simulated(simulated) {
	m_current_robot_state = m_move_group.getCurrentState();
	m_joint_model_group = m_move_group.getCurrentState()->getJointModelGroup(m_move_group_name);
	m_move_group.setPlannerId("RRTkConfigDefault");
	if(nodehandle.hasParam("robotiq_hand")) {
		nodehandle.getParam("robotiq_hand", m_robotiq_hand);
	} else {
		m_robotiq_hand = false;
	}//if else
}//ROSMoveit


/**
 * Inizializza le variabili d'istanza che riguardano il plannine e il movimento
 * @param  move_group_name  il nome del Planning Group
 */
void ros_hw_utils::ROSMoveit::initPlanningAndMove(std::string move_group_name) {
	m_move_group = moveit::planning_interface::MoveGroupInterface(move_group_name);
	m_joint_model_group = m_move_group.getCurrentState()->getJointModelGroup(move_group_name);
	m_move_group.setPlannerId("RRTkConfigDefault");
}//initPlanningAndMove


/**
 * Imposta la posizione target del manipolatore
 * @param  x  coordinata x della posizione
 * @param  y  coordinata y della posizione
 * @param  z  coordinata z della posizione
 * @param  ox  coordinata x dell'orientazione
 * @param  oy  coordinata y dell'orientazione
 * @param  oz  coordinata z dell'orientazione
 * @param  ow  coordinata w dell'orientazione
 * @param  execute  indica se eseguire o no lo spostamento del manipolatore verso la posizione target (default = false)
 * @return  true se l'esecuzione e' stata eseguita con successo, false se non e' stata eseguita o se c'e' stato un errore
 */
bool ros_hw_utils::ROSMoveit::setPose(double x, double y, double z, double ox, double oy, double oz, double ow, bool execute) {
	geometry_msgs::Pose pose;
	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = z;
	pose.orientation.x = ox;
	pose.orientation.y = oy;
	pose.orientation.z = oz;
	pose.orientation.w = ow;

	return setPose(pose, execute);
}//setPose


/**
 * Imposta la posizione target del manipolatore
 * @param  pose  la posizione target
 * @param  execute  indica se eseguire o no lo spostamento del manipolatore verso la posizione target (default = false)
 * @return  true se l'esecuzione e' stata eseguita con successo, false se non e' stata eseguita o se c'e' stato un errore
 */
bool ros_hw_utils::ROSMoveit::setPose(geometry_msgs::Pose pose, bool execute) {
        m_move_group.setStartState(*m_move_group.getCurrentState());
	m_move_group.setPoseTarget(pose);

	bool success;
	if(execute) {
		m_move_group.setNumPlanningAttempts(10);
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		//pianifichiamo
		success = (m_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");

		if(!success) {
			m_move_group.setPlanningTime(10);
			m_move_group.setPlannerId("RRTConnectkConfigDefault");
			success = (m_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
			ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");
		}//if

		std::cout << "Planning done" << std::endl;
		//eseguiamo
		m_move_group.move();
		
		std::cout << "Movimento eseguito" << std::endl;
	}//if execute

	return success;
}//setPose


/**
 * Imposta la posizione target del manipolatore assegnando una serie di punti intermedi per cui passare (Cartesian Path)
 * @param  waypoints  i punti per cui la traiettoria deve passare
 * @param  execute  indica se eseguire o no lo spostamento del manipolatore verso la posizione target seguendo i punti intermedi (default = false)
 * @return  true se l'esecuzione e' stata eseguita con successo, false se non e' stata eseguita o se c'e' stato un errore
 */
bool ros_hw_utils::ROSMoveit::setPoseWithCartesianPath(std::vector<geometry_msgs::Pose> waypoints, bool execute) {
    m_move_group.setStartState(*m_move_group.getCurrentState());
    moveit_msgs::RobotTrajectory trajectory;
    double jump_threshold = 0.0;
    double eef_step = 0.01;
    m_move_group.setNumPlanningAttempts(10);
    double fraction = m_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    if (fraction < 0.8) {
		m_move_group.setPlannerId("RRTConnectkConfigDefault");
		m_move_group.setNumPlanningAttempts(10);
		m_move_group.setPlanningTime(5);
		fraction = m_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
     }//if

    bool success = false;
    if(execute) {
		robot_trajectory::RobotTrajectory rt(m_move_group.getCurrentState()->getRobotModel(), m_move_group_name);
		rt.setRobotTrajectoryMsg(*m_move_group.getCurrentState(), trajectory);
		trajectory_processing::IterativeParabolicTimeParameterization iptp;
		bool time_success = iptp.computeTimeStamps(rt);
		rt.getRobotTrajectoryMsg(trajectory);

		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		my_plan.trajectory_ = trajectory;
		success = (m_move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }//if

	return success;
}//setPoseWithCartesianPath


/**
 * Imposta lo stato target dei giunti del manipolatore
 * @param  joint_group_positions  lo stato dei giunti da applicare
 * @param  execute  indica se eseguire o no lo spostamento del manipolatore utilizzando i nuovi stati dei giunti (default = false)
 * @return  true se l'esecuzione e' stata eseguita con successo, false se non e' stata eseguita o se c'e' stato un errore
 */
bool ros_hw_utils::ROSMoveit::setPoseWithJoints(std::vector<double> joint_group_positions, bool execute) {
	std::vector<double> old_joint_group_positions;
	m_current_robot_state->copyJointGroupPositions(m_joint_model_group, old_joint_group_positions);
	for(int i = 0;i < joint_group_positions.size();i++) {
		old_joint_group_positions[i] = joint_group_positions[i];
	}//for
	m_move_group.setJointValueTarget(old_joint_group_positions);

	bool success = false;
	if(execute) {
		m_move_group.setNumPlanningAttempts(10);
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		//success = (m_move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		success = (m_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		m_move_group.move();
	}//if
	
	return success;
}//setPoseWithJoints


/**
 * Crea un muro
 * @param  id  l'identificatore del muro da creare
 * @param  pose  la posizione del muro (indica il suo punto centrale)
 * @param  dim1  la prima dimensione del muro
 * @param  dim2  la seconda dimensione del muro
 * @param  dim3  la terza dimensione del muro (altezza)
 * @param  addAsCollisionObject  indica se aggiungere il muro tra gli oggetti di collisione (default = true)
 * @return  l'oggetto di collisione rappresentante il muro
 */
moveit_msgs::CollisionObject ros_hw_utils::ROSMoveit::createWall(std::string id, geometry_msgs::Pose pose, double dim1, double dim2, double dim3, bool addAsCollisionObject) {
	moveit_msgs::CollisionObject co;
	co.id = id;
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = dim1;
	primitive.dimensions[1] = dim2;
	primitive.dimensions[2] = dim3;

	co.primitives.push_back(primitive);
	co.primitive_poses.push_back(pose);
	co.operation = co.ADD;
	
	if(addAsCollisionObject) {
		addCollisionObject(co);
	}//if
	
	return co;
}//createWall


/**
 * Crea un contenitore per gli oggetti
 * @param  id  l'identificatore del contenitore da creare
 * @param  pose  la posizione del contenitore (indica il suo punto centrale)
 * @param  dim1  la prima dimensione del contenitore
 * @param  dim2  la seconda dimensione del contenitore
 * @param  dim3  la terza dimensione del contenitore (altezza)
 * @param  addAsCollisionObject  indica se aggiungere il contenitore tra gli oggetti di collisione (default = true)
 * @return  l'oggetto di collisione rappresentante il contenitore
 */
moveit_msgs::CollisionObject ros_hw_utils::ROSMoveit::createBasket(std::string id, geometry_msgs::Pose pose, double dim1, double dim2, double dim3, bool addAsCollisionObject) {
	moveit_msgs::CollisionObject co;
	co.id = id;
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.30;
	primitive.dimensions[1] = 0.21;
	primitive.dimensions[2] = 0.078;

	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x = 0.357361;
	box_pose.position.y = 1.19024;
	box_pose.position.z = 0.7;

	co.primitives.push_back(primitive);
	co.primitive_poses.push_back(box_pose);
	co.operation = co.ADD;
	
	if(addAsCollisionObject) {
		addCollisionObject(co);
	}//if
	
	return co;
}//createBasket


/**
 * Cerca l'oggetto di collisione con l'identificatore dato nella lista degli oggetti di collisione
 * @param  id  l'identificatore dell'oggetto di collisione
 * @return  l'indice dell'oggetto di collisione corrispondente trovato, -1 se non trovato
 */
int ros_hw_utils::ROSMoveit::getCollisionObjectIndexById(std::string id) {
	for(int i = 0;i < m_collision_objects.size();i++) {
		if(id.compare(m_collision_objects[i].id) == 0) {
			std::cout << "COLLISION OBJECT TROVATO: " << m_collision_objects[i].id << " IN POSIZIONE " << i << std::endl;
			return i;
		}//if
	}//for
	std::cout << "COLLISION OBJECT NON TROVATO: " << id << std::endl;
	return -1;
}//getCollisionObjectIndexById


/**
 * Aggiunge l'oggetto alla lista degli oggetti di collisione
 * @param  co  l'oggetto di collisione da aggiungere
 * @param  apply  aggiunge l'oggetto indicato nella scena (default = false)
 * @return  l'identificatore dell'oggetto di collisione
 */
std::string ros_hw_utils::ROSMoveit::addCollisionObject(moveit_msgs::CollisionObject co, bool apply) {
	m_collision_objects.push_back(co);
	if(apply)
		applyCollisionObjectsToScene();
	return co.id;
}//addCollisionObject


/**
 * Aggiunge l'oggetto alla lista degli oggetti di collisione
 * @param  m  la mesh da cui si ottiene l'oggetto di collisione da aggiungere
 * @param  id  l'identificatore da assegnare all'oggetto di collisione
 * @param  pose  la posizione dell'oggetto di collisione
 * @param  apply  aggiunge l'oggetto indicato nella scena (default = false)
 * @return  l'identificatore dell'oggetto di collisione
 */
std::string ros_hw_utils::ROSMoveit::addCollisionObject(const shapes::Mesh* m, std::string id, geometry_msgs::Pose pose, bool apply) {
	moveit_msgs::CollisionObject co;
    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
    co.meshes.resize(1);
    co.mesh_poses.resize(1);
    co.meshes[0] = mesh;
	co.id = id;
    co.header.frame_id = "world";
	co.mesh_poses[0] = pose;
    co.meshes.push_back(mesh);
    co.mesh_poses.push_back(co.mesh_poses[0]);
    co.operation = co.ADD;
	
	return addCollisionObject(co, apply);
}//addCollisionObject


/**
 * Aggiunge gli oggetti di collisione alla scena
 * @return  il numero di oggetti di collisione aggiunti alla scena
 */
int ros_hw_utils::ROSMoveit::applyCollisionObjectsToScene() {
	m_planning_scene_interface.addCollisionObjects(m_collision_objects);
	
	return m_collision_objects.size();
}//applyCollisionObjectsToScene


/**
 * Rimuove un oggetto di collisione dalla scena
 * @param  id  l'identificatore dell'oggetto di collisione da rimuovere
 * @param  apply  rimuovi l'oggetto di collisione anche dalla scena oppure no (default = false)
 * @return  l'oggetto di collisione rimosso
 */
moveit_msgs::CollisionObject ros_hw_utils::ROSMoveit::removeCollisionObject(std::string id, bool apply) {
	std::vector<std::string> coIdsToRemove;

	// Salvataggio della lista attuale degli oggetti di collisione
	std::vector<std::string> currentCoIds(m_collision_objects.size());
	for(int i = 0;i < currentCoIds.size();i++) {
		currentCoIds[i] = m_collision_objects[i].id;
	}//for

	moveit_msgs::CollisionObject co;
	int index = getCollisionObjectIndexById(id);
	if(index >= 0 && index < m_collision_objects.size()) {
		// Salvataggio dell'oggetto di collisione da eliminare in una variabile temporanea
		co = m_collision_objects[index];
		// Eliminazione dell'oggetto di collisione dalla lista
		m_collision_objects.erase(m_collision_objects.begin() + index);
		// Risoluzione di eventuale inconsistenza dell'indice dell'oggetto attaccato in base all'oggetto che viene eliminato
		if(index < m_attached_object_index) {
			m_attached_object_index--;
		} else if(index == m_attached_object_index){
			m_attached_object_index = -1;
		}//if else
		// Aggiunta dell'oggetto alla lista di quelli da eliminare
		coIdsToRemove.push_back(co.id);
		// Rimozione dell'oggetto di collisione
		m_planning_scene_interface.removeCollisionObjects(coIdsToRemove);
		// Se vero, applica la rimozione anche alla scena visualizzata
		if(apply) {
			// Rimuove tutti gli oggetti di collisione attuali
			m_planning_scene_interface.removeCollisionObjects(currentCoIds);
			// Riapplica solo quelli rimasti dopo l'eliminazione
			applyCollisionObjectsToScene();
		}//if
	}//if
	return co;
}//removeCollisionObject


/**
 * Rimuove tutti gli oggetti di collisione dalla scena
 */
void ros_hw_utils::ROSMoveit::removeCollisionObjects() {
	std::vector<std::string> currentCoIds(m_collision_objects.size());
	for(int i = 0;i < m_collision_objects.size();i++) {
		currentCoIds[i] = m_collision_objects[i].id;
	}//for
	m_collision_objects.clear();
	m_planning_scene_interface.removeCollisionObjects(currentCoIds);
	applyCollisionObjectsToScene();
}//removeCollisionObjects


/**
 * Effettua l'attach dell'oggetto di collisione al manipolatore
 * @param  co  l'oggetto di collisione di cui fare l'attach
 */
void ros_hw_utils::ROSMoveit::attachObject(moveit_msgs::CollisionObject co) {
	m_move_group.attachObject(co.id);
	if(m_simulated) {
		ros::ServiceClient client = m_nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
		gazebo_ros_link_attacher::Attach to_attach;
		to_attach.request.model_name_1 = "robot";
		to_attach.request.link_name_1 = "wrist_3_link";
		to_attach.request.model_name_2 = co.id;
		to_attach.request.link_name_2 = co.id + "_link";
		client.call(to_attach);
	}//if simulated
}//attachObject


/**
 * Effettua l'attach dell'oggetto di collisione al manipolatore
 * @param  id  l'identificatore dell'oggetto di collisione di cui fare l'attach
 */
void ros_hw_utils::ROSMoveit::attachObject(std::string id) {
	int index = getCollisionObjectIndexById(id);
	if(index >= 0 && index < m_collision_objects.size()) {
		attachObject(m_collision_objects[index]);
		// Salva l'indice dell'oggetto di collisione attaccato alla mano
		m_attached_object_index = index;
	}//if
}//attachObject


/**
 * Effettua il detach dell'oggetto di collisione dal manipolatore
 */
void ros_hw_utils::ROSMoveit::detachObject() {
	m_move_group.detachObject(m_collision_objects[m_attached_object_index].id);
	if(m_simulated) {
		ros::ServiceClient client = m_nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
		gazebo_ros_link_attacher::Attach to_detach;
		to_detach.request.model_name_1 = "robot";
		to_detach.request.link_name_1 = "wrist_3_link";
		to_detach.request.model_name_2 = m_collision_objects[m_attached_object_index].id;
		to_detach.request.link_name_2 = m_collision_objects[m_attached_object_index].id + "_link";
		client.call(to_detach);
	}//if simulated

	m_attached_object_index = -1;
}//detachObject


/**
 * Indica se c'e' un oggetto attaccato al manipolatore
 * @return  true se c'e' un oggetto attaccato al manipolatore, false altrimenti
 */
bool ros_hw_utils::ROSMoveit::hasObjectAttached() {
	return m_attached_object_index != -1;
}//hasObjectAttached


/**
 * Restituisce l'oggetto di collisione attaccato al manipolatore, se esiste
 * @return  l'oggetto di collisione attaccato al manipolatore
 */
moveit_msgs::CollisionObject ros_hw_utils::ROSMoveit::getAttachedObject() {
	if(hasObjectAttached())
		return m_collision_objects[m_attached_object_index];
}//getAttachedObject


/**
 * Imposta l'angolo di rotazione dell'end effector
 * @param  angle  l'angolo di cui ruotare l'end effector
 * @param  execute  indica se eseguire o no la rotazione dell'end effector (default = false)
 * @return  il vettore contenente lo stato dei giunti del manipolatore
 */
std::vector<double> ros_hw_utils::ROSMoveit::setEndEffectorOrientation(double angle, bool execute) {
	std::vector<double> positions;
	m_joint_model_group = m_move_group.getCurrentState()->getJointModelGroup(m_move_group_name);
	m_move_group.getCurrentState()->copyJointGroupPositions(m_joint_model_group, positions);
	positions[5] = angle;
	m_move_group.setJointValueTarget(positions);

	if(execute) {
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		bool success = (m_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("End effector rotation", "Visualizing plan 1 (end effector rotation) %s", success ? "SUCCESS" : "FAILED");
		m_move_group.move();
	}//execute
	
	return positions;
}//setEndEffectorOrientation


/**
 * Ottiene gli angoli di rotazione dell'end effector
 * @return  l'orientazione dell'end effector come roll-pitch-yaw
 */
std::vector<double> ros_hw_utils::ROSMoveit::getEndEffectorRPY() {
	m_joint_model_group = m_move_group.getCurrentState()->getJointModelGroup(m_move_group_name);
	return m_move_group.getCurrentRPY();
}//getEndEffectorRPY


/**
 * Ottiene la pose attuale dell'end effector
 * @return  la pose dell'end effector (posizione e orientazione in quaternioni)
 */
geometry_msgs::Pose ros_hw_utils::ROSMoveit::getEndEffectorPose() {
	m_joint_model_group = m_move_group.getCurrentState()->getJointModelGroup(m_move_group_name);
	return m_move_group.getCurrentPose().pose;
}//getEndEffectorPose


/**
 * Funzione di servizio per la gestione del gripper Robotiq
 * @param  gripper_state  lo stato del gripper
 */
void ros_hw_utils::ROSMoveit::setGripperState(robotiq_s_model_control::SModel_robot_output gripper_state) {
	ros::Publisher pub = m_nh.advertise<robotiq_s_model_control::SModel_robot_output>("/robotiq_hands/l_hand/SModelRobotOutput", 10, true);
	sleep(1);
	while(ros::ok()){
		ros::Rate r(100);
		while(pub.getNumSubscribers() == 0) {
			r.sleep();
		}//while
		pub.publish(gripper_state);
		sleep(1);
		break;
	}//while
}//manageGripper


/**
 * Apre il gripper Robotiq del manipolatore
 */
void ros_hw_utils::ROSMoveit::openGripper() {
	ROS_INFO("Opening Gripper");
	if(!m_robotiq_hand) {
		int value = 10;
        std::cout << "Inserire il valore di apertura della mano (10-120): ";
        std::cin >> value;
		value = (value < 10) ? 10 : (value > 120) ? 120 : value;
		std::fstream f;
		f.open("/dev/ttyACM0", std::fstream::out | std::fstream::app);
		if(f.is_open()) {
			f << value;
			f.close();
		}//if
	} else {
		robotiq_s_model_control::SModel_robot_output state;
		state.rACT= 1; state.rMOD= 0; state.rGTO= 1; state.rATR= 0; state.rGLV= 0; state.rICF= 0; state.rICS= 0; state.rPRA= 0; state.rSPA= 0; state.rFRA= 0; state.rPRB= 0; state.rSPB= 0; state.rFRB= 255; state.rPRC= 0; state.rSPC= 0; state.rFRC= 0; state.rPRS= 0; state.rSPS= 0; state.rFRS= 0;
		setGripperState(state);
	}//if else
}//openGripper


/**
 * Chiude il gripper Robotiq del manipolatore
 */
void ros_hw_utils::ROSMoveit::closeGripper() {
	ROS_INFO("Closing Gripper");
	if(!m_robotiq_hand) {
		int value = 120;
        std::cout << "Inserire il valore di chiusura della mano (10-120): ";
        std::cin >> value;
		value = (value < 10) ? 10 : (value > 120) ? 120 : value;
		std::fstream f;
		f.open("/dev/ttyACM0", std::fstream::out | std::fstream::app);
		if(f.is_open()) {
			f << value;
			f.close();
		}//if
	} else {
		robotiq_s_model_control::SModel_robot_output state;
		state.rACT= 1; state.rMOD= 0; state.rGTO= 1; state.rATR= 0; state.rGLV= 0; state.rICF= 0; state.rICS= 0; state.rPRA= 255; state.rSPA= 255; state.rFRA= 150; state.rPRB= 0; state.rSPB= 0; state.rFRB= 0; state.rPRC= 0; state.rSPC= 0; state.rFRC= 0; state.rPRS= 0; state.rSPS= 0; state.rFRS= 0;
		setGripperState(state);
	}//if else
}//closeGripper


/**
 * Scrive un carattere sulla porta seriale
 * @param  cmd[]  l'array di caratteri da scrivere
 * @param  cmd_size  la dimensione dell'array da scrivere (senza contare il terminatore di stringa \0)
 * @return  true se e' stato scritto almeno un carattere, false altrimenti
 */
bool ros_hw_utils::ROSMoveit::writeToSerial(const unsigned char cmd[], const unsigned int cmd_size) {
	int PORT = open("/dev/ttyACM0", O_RDWR| O_NOCTTY);
	struct termios tty;
	struct termios tty_old;
	memset (&tty, 0, sizeof tty);

	/* Error Handling */
	if ( tcgetattr ( PORT, &tty ) != 0 ) {
		std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
	}

	/* Save old tty parameters */
	tty_old = tty;

	/* Set Baud Rate */
	cfsetospeed (&tty, (speed_t)B9600);
	cfsetispeed (&tty, (speed_t)B9600);

	/* Setting other Port Stuff */
	tty.c_cflag     &=  ~PARENB;            // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;

	tty.c_cflag     &=  ~CRTSCTS;           // no flow control
	tty.c_cc[VMIN]   =  1;                  // read doesn't block
	tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

	/* Make raw */
	cfmakeraw(&tty);

	/* Flush Port, then applies attributes */
	tcflush( PORT, TCIFLUSH );
	if ( tcsetattr ( PORT, TCSANOW, &tty ) != 0) {
		std::cerr << "Error " << errno << " from tcsetattr" << std::endl;
	}

	int n_written = write(PORT, cmd, cmd_size);
	return n_written > 0;
}//writeToSerial