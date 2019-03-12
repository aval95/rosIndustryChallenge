#include "g06_perception/ROSUtils.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <algorithm>

#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <fstream> 
#include <string>

// Servizi
#include <g06_fsm/PerceptionReady.h>
#include <g06_fsm/PerceptionToFSM.h>

// Indica se l'esecuzione avviene in simulazione oppure no
bool simulated = true;

// Pubblicazione continua degli oggetti rilevati oppure uso del servizio per comunicare con la FSM
bool continuousPublishing = false;

// Istanza usata per accedere alle librerie di utilita'
ros_hw_utils::ROSUtils* utils;

// Nome del file di output
const std::string FILENAME = "results.txt";



/**
 * Scrive l'ID, la posizione e l'orientazione dell'oggetto su un file di testo
 * @param  id  l'identificatore numerico dell'oggetto
 * @param  pos  la posizione (coordinate x, y, z) dell'oggetto
 * @param  rot  l'orientazione dell'oggetto
 * @return  true se la scrittura e' andata a buon fine, false altrimenti
 */
bool writeToFile(int id, geometry_msgs::Point pos, geometry_msgs::Quaternion rot) {
    std::fstream f;
    // Il file viene salvato nella cartella del workspace
    f.open(FILENAME, std::fstream::out | std::fstream::app);
    if(f.is_open()) {
        // Lo stream del file e' aperto
        f << "ID = " << id << " (" << utils->convertIdToFrameId(id) << ")" << std::endl;
        f << "Position: x = " << pos.x << " y = " << pos.y << " z = " << pos.z << std::endl;
        f << "Orientation: x = " << rot.x << " y = " << rot.y << " z = " << rot.z << " w = " << rot.w << std::endl;
        f << std::endl;
        f.close();
        return true;
    } else {
        // Lo stream del file non e' stato aperto correttamente
        std::cerr << "Error opening output file";
        return false;
    }//if else
}//writeToFile


/**
 * Scrive l'ID, la posizione e l'orientazione dell'oggetto su console
 * @param  id  l'identificatore numerico dell'oggetto
 * @param  pos  la posizione (coordinate x, y, z) dell'oggetto
 * @param  rot  l'orientazione dell'oggetto
 */
void writeToConsole(int id, geometry_msgs::Point pos, geometry_msgs::Quaternion rot) {
    printf("id = %d\n", id);
    printf(" position x = %f\n", pos.x);
    printf(" position y = %f\n", pos.y);
    printf(" position z = %f\n", pos.z);
    printf(" rotation x = %f\n", rot.x);
    printf(" rotation y = %f\n", rot.y);
    printf(" rotation z = %f\n", rot.z);
    printf(" rotation w = %f\n", rot.w);
    printf("\n");
}//writeToConsole


/**
 * Punto di inizio dell'esecuzione
 * @param  argc  numero di parametri
 * @param  argv  stringhe rappresentanti i parametri
 * @return  il codice di stato alla fine dell'esecuzione
 */
int main(int argc, char **argv) {

    // Inizializzazione ROS
    ros::init(argc, argv, "hw1");

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

    // Controllo disponibilita' parametro continuousPublishing (se non presente, di default e' true)
    if(nh.hasParam("cont")) {
		nh.getParam("cont", continuousPublishing);
	}//if

    // Inizializzazione della libreria di utilita'
    utils = new ros_hw_utils::ROSUtils(nh, simulated);

    // Controllo disponibilita' parametro frame_ids rappresentante l'elenco dei frame_id richiesti
    // (se non disponibile, si considera che l'esecuzione sia avvenuta con rosrun invece che con roslaunch
    // e quindi si vanno a leggere gli eventuali parametri dati da riga di comando)
    int numFrameIds;
    std::vector<std::string> vectFrameIds;
    if(nh.hasParam("frame_ids")) {
        std::string frame_ids_param;
        nh.getParam("frame_ids", frame_ids_param);
        std::string delimiter = ",";
        size_t pos = 0;
        std::string token;
        while ((pos = frame_ids_param.find(delimiter)) != std::string::npos) {
            token = frame_ids_param.substr(0, pos);
            vectFrameIds.push_back(token);
            frame_ids_param.erase(0, pos + delimiter.length());
        }//while
        vectFrameIds.push_back(frame_ids_param);
        numFrameIds = vectFrameIds.size();
    } else {
        numFrameIds = argc - 1;
        vectFrameIds = std::vector<std::string>(argv + 1, argv + argc + 1);
    }//if else

    do {

        // Comunicazione del proprio stato "Pronto" al server
        ros::ServiceClient clientPerceptionReady = nh.serviceClient<g06_fsm::PerceptionReady>("/PerceptionReady");
        g06_fsm::PerceptionReady servicePR;
        servicePR.request.iamready = 1;
        if(clientPerceptionReady.call(servicePR)) {
            if(servicePR.response.received == 1) {
                ROS_INFO_STREAM("Tutto ok! PerceptionReady");
            } else {
                ROS_INFO_STREAM("Non ricevuto! PerceptionReady");
            }//if else
        } else {
            ROS_ERROR_STREAM("Errore! PerceptionReady");
        }//if else

        // Rilevazione dei tag ed elaborazione per renderli disponibili su console, su file e agli altri nodi tramite pubblicazione su topic

        std::vector<apriltags_ros::AprilTagDetection> vectDet;
        std::vector<apriltags_ros::AprilTagDetection> vectDetectedAndRequested;
        if(ros::ok()) {
            // Attende la pubblicazione di un messaggio nel topic /tag_detections
            apriltags_ros::AprilTagDetectionArrayConstPtr arrayDetections;
            arrayDetections = ros::topic::waitForMessage<apriltags_ros::AprilTagDetectionArray>("/tag_detections", nh);

            if(arrayDetections.use_count()==0) {
                // Timeout scaduto, nessun messaggio ricevuto
                ROS_WARN("Timeout getting '/tag_detections' message ");
                return 1;
            } else {
                // Messaggio ricevuto
                ROS_INFO("Received '/tag_detections' message ");
                vectDet = arrayDetections->detections;
                std::cout << "Number of objects detected: " << vectDet.size() << std::endl << std::endl;
            }//if else
        }//if

        // Associa ad ogni oggetto identificato il suo frame_id ed applica l'offset correttivo alle coordinate
        for(int j = 0;j < vectDet.size();j++) {
            vectDet[j] = utils->applyOffset(vectDet[j]);
            vectDet[j].pose.header.frame_id = utils->convertIdToFrameId(vectDet[j].id);
        }//for

        // Il massimo numero di elementi che possono essere messi davanti alla telecamera e' 16
        if(numFrameIds <= 16) {
            for(int i = 0;i < numFrameIds;i++) {
                int id = utils->convertFrameIdToId(vectFrameIds[i]);
                if(id == -1) {
                    std::cout << vectFrameIds[i] << " is not a valid frame_id" << std::endl << std::endl;
                } else {
                    bool found = false;
                    for(int j = 0;j < vectDet.size();j++) {
                        if(id == vectDet[j].id) {
                            vectDetectedAndRequested.push_back(vectDet[j]);
                            std::cout << id << " (" << utils->convertIdToFrameId(id) << ")" << " found!" << std::endl;
                            writeToConsole(vectDet[j].id, vectDet[j].pose.pose.position, vectDet[j].pose.pose.orientation);
                            writeToFile(vectDet[j].id, vectDet[j].pose.pose.position, vectDet[j].pose.pose.orientation);
                            found = true;
                            break;
                        }//if
                    }//for
                    if(!found) {
                        std::cout << id << " (" << utils->convertIdToFrameId(id) << ")" << " not found" << std::endl << std::endl;
                    }//if
                }//if else
            }//for
        } else {
            std::cerr << "*** Too many arguments! ***";
            exit(1);
        }//if else

        // Necessario per pubblicazione dei tag in multithreading
        ros::AsyncSpinner spinner(3);
        spinner.start();

        // Pubblicazione dei tag rilevati e di quelli rilevati e richiesti
        apriltags_ros::AprilTagDetectionArray detected;
        detected.detections = vectDet;
        apriltags_ros::AprilTagDetectionArray detectedAndRequested;
        detectedAndRequested.detections = vectDetectedAndRequested;

        // Comunicazione dell'avvenuto rilevamento al server
        ros::ServiceClient clientPerceptionToFSM = nh.serviceClient<g06_fsm::PerceptionToFSM>("/PerceptionToFSM");
        g06_fsm::PerceptionToFSM serviceP2FSM;
        serviceP2FSM.request.numFound = detected.detections.size();
        serviceP2FSM.request.detections_all = detected;
        serviceP2FSM.request.numRequested = detectedAndRequested.detections.size();
        serviceP2FSM.request.detections_requested = detectedAndRequested;
        if(clientPerceptionToFSM.call(serviceP2FSM)) {
            if(serviceP2FSM.response.ack == 1) {
                ROS_INFO_STREAM("Tutto ok! PerceptionToFSM");
            } else {
                ROS_ERROR_STREAM("Errore! PerceptionToFSM");
                break;
            }//if else
        } else {
            ROS_ERROR_STREAM("Errore! PerceptionToFSM");
        }//if else

        utils->publishAllAndRequestedPoses("tag_detections_all", detected, "tag_detections_requested", detectedAndRequested, continuousPublishing);

    } while(true);

    ros::waitForShutdown();
    
    return 0;
}//main