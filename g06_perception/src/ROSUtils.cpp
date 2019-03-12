#include <g06_perception/ROSUtils.h>

/**
 * Costruttore, inizializza le variabili d'istanza
 * @param  nodehandle  l'oggetto NodeHandle relativo al nodo corrente
 * @param  simulated  indica se si sta eseguendo il nodo in simulazione oppure no
 */
ros_hw_utils::ROSUtils::ROSUtils(ros::NodeHandle nodehandle, bool simulated) : m_nh(nodehandle), m_simulated(simulated) {

    // Inizializzazione dell'array contenente la corrispondenza id-frame_id
    // Si poteva usare anche una mappa per velocizzare la conversione di frame_id in id (ricerca in O(1))
    // ma avendo solo 16 elementi il guadagno in termini di complessita' computazionale e' trascurabile
    m_frame_ids = std::vector<std::string>(NUM_OBJECTS);
    m_frame_ids[0] = "red_cube_1";
    m_frame_ids[1] = "red_cube_2";
    m_frame_ids[2] = "red_cube_3";
    m_frame_ids[3] = "red_cube_4";
    m_frame_ids[4] = "yellow_cyl_1";
    m_frame_ids[5] = "yellow_cyl_2";
    m_frame_ids[6] = "green_triangle_1";
    m_frame_ids[7] = "green_triangle_2";
    m_frame_ids[8] = "green_triangle_3";
    m_frame_ids[9] = "blue_cube_1";
    m_frame_ids[10] = "blue_cube_2";
    m_frame_ids[11] = "blue_cube_3";
    m_frame_ids[12] = "blue_cube_4";
    m_frame_ids[13] = "red_triangle_1";
    m_frame_ids[14] = "red_triangle_2";
    m_frame_ids[15] = "red_triangle_3";

    m_model_ids = std::vector<std::string>(NUM_OBJECTS);
    m_model_ids[0] = "cube1";
    m_model_ids[1] = "cube2";
    m_model_ids[2] = "cube3";
    m_model_ids[3] = "cube4";
    m_model_ids[4] = "Hexagon0";
    m_model_ids[5] = "Hexagon1";
    m_model_ids[6] = "Triangle0";
    m_model_ids[7] = "Triangle1";
    m_model_ids[8] = "Triangle2";
    m_model_ids[9] = "blue_cube_1";
    m_model_ids[10] = "blue_cube_2";
    m_model_ids[11] = "blue_cube_3";
    m_model_ids[12] = "blue_cube_4";
    m_model_ids[13] = "red_triangle_1";
    m_model_ids[14] = "red_triangle_2";
    m_model_ids[15] = "red_triangle_3";	

}//ROSUtils


/**
 * Funzione utilizzata per caricare i parametri di navigazione da file
 * @return  la mappa contenente i parametri
 */
std::map<std::string, double> ros_hw_utils::ROSUtils::loadConfigParams() {
    std::vector<std::string> paramsName;
    m_nh.getParamNames(paramsName);

    // Mantiene solo quelli privati
    for(int i = 0;i < paramsName.size();i++) {
        if(paramsName[i].find("/g06_navigation_node/") == std::string::npos) {
            paramsName.erase(paramsName.begin() + i);
            i--;
        }//if
    }//for

    // Inserimento dei parametri e dei loro valori nella mappa
    for(std::string name : paramsName) {
        double param;
        m_nh.getParam(name, param);
        name = name.substr(21, name.size() - 21);
        m_params.insert(std::pair<std::string, double> (name, param));
    }//for
}//loadConfigParams


/**
 * Funzione per richiamare i parametri per nome
 * @param  name  il nome del parametro
 * @return  il valore del parametro
 */
double ros_hw_utils::ROSUtils::getParam(std::string name) {
    return m_params[name];
}//getParam


/** Restituisce il tipo di forma primitiva (cubo, poliedro a facce triangolari, cilindro a basi esagonali)
 * @param  id  l'identificatore numerico dell'oggetto
 * @return  l'identificatore del tipo di forma primitiva (0 = cubo, 1 = cilindro, 2 = triangolo), -1 se il parametro id non è valido
 */
int ros_hw_utils::ROSUtils::getPrimitiveShape(int id) {
    if((id >= 0 && id <=3) || (id >=9 && id <= 12)) {
        return SHAPE_CUBE;
    } else if(id== 4 || id == 5) {
        return SHAPE_CYLINDER;
    } else if((id >=6 && id <= 8) || (id >= 13 && id <=15)) {
        return SHAPE_TRIANGLE;
    } else {
        return -1;
    }//if else
}//getPrimitiveShape


/**
 * Converte l'id dell'oggetto (int) nel corrispondente frame_id (string)
 * @param  id  l'identificatore numerico dell'oggetto
 * @return  il frame_id dell'oggetto avente id come identificatore numerico
 */
std::string ros_hw_utils::ROSUtils::convertIdToFrameId(int id) {
    if(id >= 0 && id <= 15) {
        return m_frame_ids[id];
    }//if
    return "Not identified";
}//convertIdToFrameId


/**
 * Converte il frame_id dell'oggetto (string) nel corrispondente id (int)
 * @param  frame_id  il frame_id dell'oggetto
 * @return  l'identificatore numerico dell'oggetto
 */
int ros_hw_utils::ROSUtils::convertFrameIdToId(std::string frame_id) {
    for(int i = 0;i < m_frame_ids.size();i++) {
        if(m_frame_ids[i].compare(frame_id) == 0) {
            return i;
        }//if
    }//for
    return -1;
}//convertFrameIdToId


/**
 * Restituisce il nome del modello per la simulazione dato l'identificatore
 * @param  id  l'identificatore numerico dell'oggetto
 * @return  il nomde del modello corrispondente
 */
std::string ros_hw_utils::ROSUtils::getModelNameById(int id) {
    if(id >= 0 && id <= 15) {
        return m_model_ids[id];
    }//if
    return "Not identified";
}//getModelNameById


/**
 * Applica un'offset per compensare l'errore nella rilevazione della posizione degli AprilTag (offset diverso per ogni tipo di oggetto)
 * @param  detection  informazioni riguardo l'oggetto trovato
 * @return  le informazioni riguardo l'oggetto trovato, con la posizione aggiornate tendendo conto dell'offset
 */
apriltags_ros::AprilTagDetection ros_hw_utils::ROSUtils::applyOffset(const apriltags_ros::AprilTagDetection detection) {
    double offsetX;
    double offsetY;
    if(m_simulated) {
        offsetX = -(detection.size / 8);
        offsetY = -(detection.size / 2);
    } else {
        offsetX = +(detection.size / 8);
        offsetY = +(detection.size / 2);
    }//if else
    apriltags_ros::AprilTagDetection detectionWithOffset = detection;
    detectionWithOffset.pose.pose.position.x += offsetX;
    detectionWithOffset.pose.pose.position.y += offsetY;
    return detectionWithOffset;
}//applyOffset


/**
 * Crea un oggetto Pose a partire dalle posizioni e orientazioni
 * @param  x  coordinata x della posizione
 * @param  y  coordinata y della posizione
 * @param  z  coordinata z della posizione
 * @param  ox  coordinata x dell'orientazione
 * @param  oy  coordinata y dell'orientazione
 * @param  oz  coordinata z dell'orientazione
 * @param  ow  coordinata w dell'orientazione
 * @return  l'oggetto Pose
 */
geometry_msgs::Pose ros_hw_utils::ROSUtils::createPose(double x, double y, double z, double ox, double oy, double oz, double ow) {
    geometry_msgs::Pose pose;
	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = z;
	pose.orientation.x = ox;
	pose.orientation.y = oy;
	pose.orientation.z = oz;
	pose.orientation.w = ow;
	return pose;
}//createPose


/**
 * Pubblica le posizioni degli oggetti passati come parametro
 * @param  topicName  il nome del topic in cui pubblicare le posizioni
 * @param  foundItems  l'array contenente le informazioni riguardanti gli oggetti rilevati grazie agli AprilTags
 * @param  continuePublishing  indica se continuare la pubblicazione dello stesso messaggio
 */
void ros_hw_utils::ROSUtils::publishPoses(const std::string topicName, const apriltags_ros::AprilTagDetectionArray foundItems, bool continuePublishing) {
    apriltags_ros::AprilTagDetectionArray foundItemsUpdatedTimestamp = foundItems;
    ros::Publisher pub = m_nh.advertise<apriltags_ros::AprilTagDetectionArray>(topicName, 1000);
    ros::Rate rate(1);
    if(ros::ok()) {
        do {
            for(int i = 0; i < foundItemsUpdatedTimestamp.detections.size(); foundItemsUpdatedTimestamp.detections[i++].pose.header.stamp = ros::Time::now());
            pub.publish(foundItemsUpdatedTimestamp);
            rate.sleep();
        } while(continuePublishing && ros::ok());
    }//if
}//publishPoses


/**
 * Pubblica le posizioni degli oggetti passati come parametro
 * @param  topicNameAll  il nome del topic in cui pubblicare le posizioni di tutti gli oggetti rilevati
 * @param  foundItems  l'array contenente le informazioni riguardanti gli oggetti rilevati grazie agli AprilTags
 * @param  topicNameRequested  il nome del topic in cui pubblicare le posizioni degli oggetti rilevati e richiesti
 * @param  foundRequestedItems  l'array contenente le informazioni riguardanti gli oggetti rilevati grazie agli AprilTags e richiesti
 * @param  continuePublishing  indica se continuare la pubblicazione dello stesso messaggio
 */
void ros_hw_utils::ROSUtils::publishAllAndRequestedPoses(const std::string topicNameAll, const apriltags_ros::AprilTagDetectionArray foundItems, const std::string topicNameRequested, const apriltags_ros::AprilTagDetectionArray foundRequestedItems, bool continuePublishing) {
    apriltags_ros::AprilTagDetectionArray foundItemsUpdatedTimestamp = foundItems;
    apriltags_ros::AprilTagDetectionArray foundRequestedItemsUpdatedTimestamp = foundRequestedItems;
    ros::Publisher pubAll = m_nh.advertise<apriltags_ros::AprilTagDetectionArray>(topicNameAll, 1000);
    ros::Publisher pubRequested = m_nh.advertise<apriltags_ros::AprilTagDetectionArray>(topicNameRequested, 1000);
    ros::Rate rate(1);
    if(ros::ok()) {
        do {
            for(int i = 0; i < foundItemsUpdatedTimestamp.detections.size(); foundItemsUpdatedTimestamp.detections[i++].pose.header.stamp = ros::Time::now());
            for(int i = 0; i < foundRequestedItemsUpdatedTimestamp.detections.size(); foundRequestedItemsUpdatedTimestamp.detections[i++].pose.header.stamp = ros::Time::now());
            pubAll.publish(foundItemsUpdatedTimestamp);
            pubRequested.publish(foundRequestedItemsUpdatedTimestamp);
            rate.sleep();
        } while(continuePublishing && ros::ok());
    }//if
}//publishAllAndRequestedPoses


/**
 * Esegue la trasformata delle coordinate, cambiando sistema di riferimento
 * @param  original_pose  posizione originale
 * @param  toFrame  il sistema di riferimento di destinazione (default = "world")
 * @param  fromFrame  il sistema di riferimento di origine (default = "camera_rgb_optical_frame")
 * @return  la posizione ottenuta dopo la trasformata
 */
geometry_msgs::Pose ros_hw_utils::ROSUtils::doTransform(geometry_msgs::Pose original_pose, std::string toFrame, std::string fromFrame, std::string publishName) {
    tf::TransformListener listener;
    ros::Time now = ros::Time::now();

    tf::StampedTransform transform;
	if (listener.waitForTransform(toFrame, fromFrame, now, ros::Duration(5.0)))
		listener.lookupTransform(toFrame, fromFrame, now, transform);
    
    tf::Matrix3x3 rotation_matrix = tf::Matrix3x3(transform.getRotation());
    
    tf::Vector3 original_position;
    original_position.setX(original_pose.position.x);
    original_position.setY(original_pose.position.y);
    original_position.setZ(original_pose.position.z);
    tf::Vector3 transformed_position = rotation_matrix * original_position + transform.getOrigin();

    tf::Quaternion original_orientation;
    tf::quaternionMsgToTF(original_pose.orientation, original_orientation);
    tf::Quaternion transformed_orientation = (transform.getRotation() * original_orientation).normalize();

    tf::Transform transformed;
    transformed.setOrigin(transformed_position);
    transformed.setRotation(transformed_orientation);

    geometry_msgs::Pose transformed_pose;
    tf::poseTFToMsg(transformed, transformed_pose);

    tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(tf::StampedTransform(transformed, now, toFrame, "trans_" + publishName));

    return transformed_pose;
}//doTransform