#include <g06_perception/ROSUtils.h>
#include <g06_navigation/ROSNavigation.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <unistd.h>
#include <cmath>
#include <actionlib_msgs/GoalStatusArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

// Servizi
#include <g06_fsm/PerceptionReady.h>
#include <g06_fsm/ManipulationReady.h>
#include <g06_fsm/NavigationReady.h>
#include <g06_fsm/UserReady.h>
#include <g06_fsm/PerceptionToFSM.h>
#include <g06_fsm/ManipulationToFSM.h>
#include <g06_fsm/NavigationToFSM.h>
#include <g06_fsm/UserToFSM.h>

#define MAX_OBJECTS_LOAD 4

// Istanze usate per accedere alle librerie di utilita'
ros_hw_utils::ROSNavigation* navi;
ros_hw_utils::ROSUtils* utils;

// Indica se l'esecuzione avviene in simulazione oppure no
bool simulated = true;

// Abilita o meno la visualizzazione di alcune informazioni di debug aggiuntive
bool debug = false;

// Stato dei nodi
// Per iniziare l'esecuzione tutti i nodi devono informare che sono pronti
bool isPerceptionReady = false;
bool isManipulationReady = false;
bool isNavigationReady = false;
bool isUserReady = false;

// Variabili di stato che indicano a che punto si e' arrivati con l'esecuzione
// oppure contengono informazioni utili per coordinare i nodi
bool loadBaseReached = false;
bool detectionDone = false;
bool loadDone = false;

bool unloadBaseReached = false;
bool unloadDone = false;

int numFound;
apriltags_ros::AprilTagDetectionArray detections_all;
int numRequested;
apriltags_ros::AprilTagDetectionArray detections_requested;

int objectsToLoad = 7;

int startIamhere = 0;
int localIamhere = startIamhere;

geometry_msgs::PoseStamped localRobotPose;

/**
 * Funzione callback per servizio perceptionReady (per indicare la disponibilita' del nodo Perception a compiere azioni)
 * @param  req  l'insieme delle variabili facenti parte della richiesta al servizio
 * @param  res  l'insieme delle variabili facenti parte della risposta da parte del servizio
 * @return  il risultato della callback
 */
bool perceptionReady(g06_fsm::PerceptionReady::Request &req, g06_fsm::PerceptionReady::Response &res) {
    ROS_INFO_STREAM("Richiesta arrivata! PerceptionReady");
    if(req.iamready == 1) {
        isPerceptionReady = true;
    }//if
    // Attesa che Marrtino arrivi alla base di carico per fare la lettura degli AprilTags
    while(!loadBaseReached) {
        sleep(1);
    }//while
    // Marrtino e' arrivato alla base di carico, avvia le funzioni del nodo Perception
    res.received = 1;
    loadBaseReached = false;
    ROS_INFO_STREAM("Richiesta servita! PerceptionReady");
    return true;
}//perceptionReady


/**
 * Funzione callback per servizio manipulationReady (per indicare la disponibilita' del nodo Manipulation a compiere azioni)
 * @param  req  l'insieme delle variabili facenti parte della richiesta al servizio
 * @param  res  l'insieme delle variabili facenti parte della risposta da parte del servizio
 * @return  il risultato della callback
 */
bool manipulationReady(g06_fsm::ManipulationReady::Request &req, g06_fsm::ManipulationReady::Response &res) {
    ROS_INFO_STREAM("Richiesta arrivata! ManipulationReady");
    if(req.iamready == 1) {
        isManipulationReady = true;
    }//if
    // Attesa che il nodo Perception effettui ed elabori le letture degli AprilTags
    while(!detectionDone) {
        sleep(1);
    }//while
    // Marrtino e' nella base di carico e gli AprilTags sono stati trovati
    res.received = 1;
    res.numFound = detections_all.detections.size();
    res.detections_all = detections_all;

    // Ad ogni giro del robot mobile, si decide quali oggetti verranno caricati dal robot manipolatore
    // Al massimo MAX_OBJECTS_LOAD oggetti verranno caricati ad ogni giro
    apriltags_ros::AprilTagDetectionArray actual_detections_requested;
    int objectsCount = 0;
    // Cerca i triangoli, verranno presi per primi (se presenti)
    for(int i = 0;i < detections_requested.detections.size();i++) {
        if(utils->getPrimitiveShape(detections_requested.detections[i].id) == utils->SHAPE_TRIANGLE && objectsCount < MAX_OBJECTS_LOAD) {
            actual_detections_requested.detections.push_back(detections_requested.detections[i]);
            objectsCount++;
        }//if
    }//for
    // Cerca i cilindri, verranno presi dopo i triangoli (se presenti)
    for(int i = 0;i < detections_requested.detections.size();i++) {
        if(utils->getPrimitiveShape(detections_requested.detections[i].id) == utils->SHAPE_CYLINDER && objectsCount < MAX_OBJECTS_LOAD) {
            actual_detections_requested.detections.push_back(detections_requested.detections[i]);
            objectsCount++;
        }//if
    }//for
    // Cerca i cilindri, verranno presi dopo i triangoli (se presenti)
    for(int i = 0;i < detections_requested.detections.size();i++) {
        if(utils->getPrimitiveShape(detections_requested.detections[i].id) == utils->SHAPE_CUBE && objectsCount < MAX_OBJECTS_LOAD) {
            actual_detections_requested.detections.push_back(detections_requested.detections[i]);
            objectsCount++;
        }//if
    }//for

    res.numRequested = actual_detections_requested.detections.size();
    res.detections_tograb = actual_detections_requested;
    res.robotPose = localRobotPose;
    detectionDone = false;
    ROS_INFO_STREAM("Richiesta servita! ManipulationReady");
    return true;
}//manipulationReady


/**
 * Funzione callback per servizio navigationReady (per indicare la disponibilita' del nodo Navigation a compiere azioni)
 * @param  req  l'insieme delle variabili facenti parte della richiesta al servizio
 * @param  res  l'insieme delle variabili facenti parte della risposta da parte del servizio
 * @return  il risultato della callback
 */
bool navigationReady(g06_fsm::NavigationReady::Request &req, g06_fsm::NavigationReady::Response &res) {
    ROS_INFO_STREAM("Richiesta arrivata! NavigationReady");
    if(req.iamready == 1) {
        isNavigationReady = true;
    }//if
    // Attesa che tutti gli altri nodi siano pronti ad operare
    while(!(isPerceptionReady && isManipulationReady && isNavigationReady && isUserReady)) {
        usleep(100);
    }//while
    res.received = 1;
    // Posizione di inizio di Marrtino
    res.startiamhere = startIamhere;
    ROS_INFO_STREAM("Richiesta servita! NavigationReady");
    return true;
}//navigationReady


/**
 * Funzione callback per servizio userReady (per indicare la disponibilita' del nodo User a compiere azioni)
 * @param  req  l'insieme delle variabili facenti parte della richiesta al servizio
 * @param  res  l'insieme delle variabili facenti parte della risposta da parte del servizio
 * @return  il risultato della callback
 */
bool userReady(g06_fsm::UserReady::Request &req, g06_fsm::UserReady::Response &res) {
    ROS_INFO_STREAM("Richiesta arrivata! UserReady");
    if(req.iamready == 1) {
        isUserReady = true;
    }//if
    // Attesa che Marrtino arrivi alla base di scarico
    while(!unloadBaseReached) {
        sleep(1);
    }//while
    res.received = 1;
    unloadBaseReached = false;
    ROS_INFO_STREAM("Richiesta servita! UserReady");
    return true;
}//userReady


/**
 * Funzione callback per servizio navigationToFSM
 * Il nodo Navigation informa la FSM circa la posizione (valore di iamhere) di Marrtino e in base a quella la FSM
 * sa se deve risvegliare il nodo Manipulation, quello User o nessuno nel caso in cui non ci sia piu' nulla da trasportare
 * @param  req  l'insieme delle variabili facenti parte della richiesta al servizio
 * @param  res  l'insieme delle variabili facenti parte della risposta da parte del servizio
 * @return  il risultato della callback
 */
bool navigationToFSM(g06_fsm::NavigationToFSM::Request &req, g06_fsm::NavigationToFSM::Response &res) {
    ROS_INFO_STREAM("Richiesta arrivata! NavigationToFSM");
    localIamhere = req.iamhere;
    localRobotPose = req.robotPose;
    if(localIamhere == 3) {
        // Base di carico
        loadBaseReached = true;
        while(!loadDone) {
            sleep(1);
        }//while
        loadDone = false;
        res.gonext = 1;
    } else if(localIamhere == 8) {
        // Base di scarico, potrebbe anche non servire continuare i giri
        unloadBaseReached = true;
        while(!unloadDone) {
            sleep(1);
        }//while
        unloadDone = false;
        res.gonext = (objectsToLoad > 0);
    } else {
        // Altre posizioni che non richiedono interazione con altri nodi
        loadBaseReached = false;
        unloadBaseReached = false;
        res.gonext = 1;
    }//if else
    res.ack = 1;
    ROS_INFO_STREAM("Richiesta servita! NavigationToFSM");
    return true;
}//navigationToFSM


/**
 * Funzione callback per servizio perceptionToFSM
 * Il nodo Perception informa la FSM circa gli oggetti che ci sono sul tavolo e quali tra loro sono richiesti
 * @param  req  l'insieme delle variabili facenti parte della richiesta al servizio
 * @param  res  l'insieme delle variabili facenti parte della risposta da parte del servizio
 * @return  il risultato della callback
 */
bool perceptionToFSM(g06_fsm::PerceptionToFSM::Request &req, g06_fsm::PerceptionToFSM::Response &res) {
    ROS_INFO_STREAM("Richiesta arrivata! PerceptionToFSM");
    // Letture elaborate, con eventuali offset, che provengono dal nodo Perception
    numFound = req.numFound;
    detections_all = req.detections_all;
    numRequested = req.numRequested;
    detections_requested = req.detections_requested;
    objectsToLoad = numRequested;
    res.ack = 1;
    detectionDone = true;
    isPerceptionReady = false;
    ROS_INFO_STREAM("Richiesta servita! PerceptionToFSM");
    return true;
}//perceptionToFSM


/**
 * Funzione callback per servizio manipulationToFSM
 * Il nodo Manipulation informa la FSM circa il numero di oggetti caricati su Marrtino, 
 * la FSM risponde indicando quanti oggetti mancano ancora da caricare nei prossimi giri
 * @param  req  l'insieme delle variabili facenti parte della richiesta al servizio
 * @param  res  l'insieme delle variabili facenti parte della risposta da parte del servizio
 * @return  il risultato della callback
 */
bool manipulationToFSM(g06_fsm::ManipulationToFSM::Request &req, g06_fsm::ManipulationToFSM::Response &res) {
    ROS_INFO_STREAM("Richiesta arrivata! ManipulationToFSM");
    if(req.done == 1) {
        loadDone = true;
        objectsToLoad -= req.numLoaded;
    } else {
        ROS_ERROR_STREAM("Carico oggetti non riuscito!");
    }//if else
    res.ack = 1;
    res.numRemaining = objectsToLoad;
    isManipulationReady = false;
    ROS_INFO_STREAM("Richiesta servita! ManipulationToFSM");
    return (req.done == 1);
}//manipulationToFSM


/**
 * Funzione callback per servizio userToFSM
 * Il nodo User contatta la FSM per indicare che lo scarico manuale e' stato effettuato oppure no
 * e di conseguenza si puo' ridare il controllo al nodo Navigation (se serve fare un ulteriore giro)
 * @param  req  l'insieme delle variabili facenti parte della richiesta al servizio
 * @param  res  l'insieme delle variabili facenti parte della risposta da parte del servizio
 * @return  il risultato della callback
 */
bool userToFSM(g06_fsm::UserToFSM::Request &req, g06_fsm::UserToFSM::Response &res) {
    ROS_INFO_STREAM("Richiesta arrivata! UserToFSM");
    if(req.done == 1) {
        unloadDone = true;
    } else {
        ROS_ERROR_STREAM("Scarico oggetti non riuscito!");
    }//if else
    res.ack = 1;
    isUserReady = false;
    ROS_INFO_STREAM("Richiesta servita! UserToFSM");
    return (req.done == 1);
}//userToFSM


/**
 * Punto di inizio dell'esecuzione
 * @param  argc  numero di parametri
 * @param  argv  stringhe rappresentanti i parametri
 * @return  il codice di stato alla fine dell'esecuzione
 */
int main(int argc, char **argv) {

    // Inizializzazione ROS
    ros::init(argc, argv, "hw4");

    // NodeHandle
    ros::NodeHandle nh;

    // Lettura parametri
    if(nh.hasParam("sim")) {
		nh.getParam("sim", simulated);
	}//if
	std::cout << "STARTS ";
	if(!simulated)
		std::cout << "NOT ";
	std::cout << "SIMULATED" << std::endl;

    if(nh.hasParam("debug")) {
		nh.getParam("debug", debug);
	}//if

    if(nh.hasParam("startiamhere")) {
		nh.getParam("startiamhere", startIamhere);
	}//if
    localIamhere = startIamhere;

    // Creazione istanze per uso della libreria di utilita'
    utils = new ros_hw_utils::ROSUtils(nh, simulated);

    // Vengono usati 9 thread, uno per la FSM ed uno per ogni servizio
    ros::MultiThreadedSpinner spinner(9);
    
    // Creazione dei servizi
    ros::ServiceServer serverManipulationReady = nh.advertiseService("/ManipulationReady", manipulationReady);
    ros::ServiceServer serverPerceptionReady = nh.advertiseService("/PerceptionReady", perceptionReady);
    ros::ServiceServer serverNavigationReady = nh.advertiseService("/NavigationReady", navigationReady);
    ros::ServiceServer serverUserReady = nh.advertiseService("/UserReady", userReady);
    ros::ServiceServer serverManipulationToFSM = nh.advertiseService("/ManipulationToFSM", manipulationToFSM);
    ros::ServiceServer serverPerceptionToFSM = nh.advertiseService("/PerceptionToFSM", perceptionToFSM);
    ros::ServiceServer serverNavigationToFSM = nh.advertiseService("/NavigationToFSM", navigationToFSM);
    ros::ServiceServer serverUserToFSM = nh.advertiseService("/UserToFSM", userToFSM);

    spinner.spin();

    return 0;
}//main