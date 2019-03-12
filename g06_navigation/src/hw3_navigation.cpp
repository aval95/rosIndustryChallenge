#include <g06_perception/ROSUtils.h>
#include <g06_navigation/ROSNavigation.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <sensor_msgs/LaserScan.h>
#include <unistd.h>
#include <cmath>
#include <algorithm>
#include <actionlib_msgs/GoalStatusArray.h>
#include <laser_line_extraction/LineSegment.h>
#include <laser_line_extraction/LineSegmentList.h>

// Servizi
#include <g06_fsm/NavigationReady.h>
#include <g06_fsm/NavigationToFSM.h>

// Istanze usate per accedere alle librerie di utilita'
ros_hw_utils::ROSNavigation* navi;
ros_hw_utils::ROSUtils* utils;

// Indica se l'esecuzione avviene in simulazione oppure no
bool simulated = true;

// Abilita o meno la visualizzazione di alcune informazioni di debug aggiuntive
bool debug = false;

// Indica se il laser e' a 360 gradi oppure no
bool is360laser = false;

// Indica il topic da utilizzare per inviare i comandi di movimento
std::string cmd_vel_topic;

// Indica se visualizzare solo cio' che rilevano i sensori invece di avviare la navigazione
bool sensors_only_mode = false;

// Indica se forzare o meno la rotazione del robot in caso di ostacoli troppo vicini
bool forceRotation = false;

// iamhere indica il punto in cui si e' nell'esecuzione delle routine di navigazione
int iamhere = 0;

/*
Come interpretare iamhere:
0 = da inizio in arena a entrata passaggio stretto, planner + recovery
1 = entrata passaggio stretto da arena
2 = passaggio stretto da arena
3 = prelievo carico
4 = entrata passaggio stretto da prelievo
5 = passaggio stretto da prelievo
6 = uscita passaggio stretto da prelievo
7 = verso stazione scarico
8 = in stazione scarico
9 = da stazione scarico a entrata passaggio stretto

Percorso = 0 1 2 3 4 5 6 7 8 9 1 2 3 4 ...
*/

// Variabile velocita' usata nelle operazioni di movimento quando non si usano i metodi di libreria
geometry_msgs::Twist vel;

// Ultimo movimento manuale eseguito durante la navigazione in iamhere 2 e 5
int last = -1;

// Indica se pubblicare o no il movimento, per evitare pubblicazioni ripetute dello stesso movimento
bool publish = false;

// Indica se si e' o meno in recovery
bool recovery = false;

// Conta il numero di volte in cui si torna in recovery subito dopo esserci usciti
int consecutiveRecoveryCount = 0;

// Conta quanti passi di navigazione sono stati eseguiti senza operazioni di recovery a dividerli
int consecutiveNoRecoveryCount = 0;

// Evita di visualizzare un messaggio di stato piu' volte
bool displayed = false;

// Variabile con funzione di mutex, serve a fare in modo che l'esecuzione di una serie di istruzioni non venga
// interrotta da un'altra esecuzione successiva della callback
bool processing = false;
bool linesProcessing = false;
bool laserProcessing = false;

// Permette di fermare la navigazione (si indica al robot che il goal e' la posizione attuale, ma non si vuole 
// ingannare la routine di controllo dato che in realta' la posizione raggiunta non e' il vero goal ma un goal "di servizio")
bool falseGoalReached = false;

/// Variabili di stato, servono a capire a che punto si e' nell'esecuzione della routine interna ad uno iamhere

// Variabili di stato per iamhere 0
bool planSent0 = false;
bool turned0 = false;
bool wallFollowed0 = false;

// Variabili di stato per iamhere 1
bool chosen1 = false;
bool wellpositioned1 = false;
bool approached1 = false;
bool turned1 = false;
bool wellpositioned12 = false;
int rot_sign1 = -1;

// Variabili di stato per iamhere 2
bool insideCorridor2 = false;

// Variabili di stato per iamhere 3
bool turned3 = false;
bool docked3 = false;

// Variabili di stato per iamhere 4
bool undocked4 = false;
bool turned4 = false;

// Variabili di stato per iamhere 6
bool positioned6 = false;
bool turned6 = false;

// Variabili di stato per iamhere 7
bool planSent7 = false;

// Variabili di stato per iamhere 8
bool chosen8 = false;
bool wellpositioned8 = false;
bool docked8 = false;
int rot_sign8 = -1;
bool rightWallFound8 = false;
bool leftWallFound8 = false;
bool turned8 = false;
bool rotated8 = false;

// Variabili di stato per iamhere 9
bool undocked9 = false;
bool rotated9 = false;
bool planSent9 = false;


/**
 * Funzione necessaria per resettare le variabili di stato alla fine di ogni giro
 */
void resetStatus() {
    last = -1;
    planSent0 = false;
    turned0 = false;
    wallFollowed0 = false;

    // Variabili di stato per iamhere 1
    chosen1 = false;
    wellpositioned1 = false;
    approached1 = false;
    turned1 = false;
    wellpositioned12 = false;
    rot_sign1 = -1;

    // Variabili di stato per iamhere 2
    insideCorridor2 = false;

    // Variabili di stato per iamhere 3
    turned3 = false;
    docked3 = false;

    // Variabili di stato per iamhere 4
    undocked4 = false;
    turned4 = false;

    // Variabili di stato per iamhere 6
    positioned6 = false;
    turned6 = false;

    // Variabili di stato per iamhere 7
    planSent7 = false;

    // Variabili di stato per iamhere 8
    chosen8 = false;
    wellpositioned8 = false;
    docked8 = false;
    rot_sign8 = -1;
    rightWallFound8 = false;
    leftWallFound8 = false;
    turned8 = false;
    rotated8 = false;

    // Variabili di stato per iamhere 9
    undocked9 = false;
    rotated9 = false;
    planSent9 = false;
}//resetStatus


/**
 * Presa del lock su entrambe le funzioni callback executionCallback e laserLineCallback
 */
void lock() {
    processing = true;
}//lock


/**
 * Presa del lock sulla funzione callback executionCallback
 */
void lockLaser() {
    laserProcessing = true;
}//lockLaser


/**
 * Presa del lock sulla funzione callback laserLineCallback
 */
void lockLines() {
    linesProcessing = true;
}//lockLines


/**
 * Rilascio del lock su entrambe le funzioni callback executionCallback e laserLineCallback
 */
void unlock() {
    processing = false;
}//unlock


/**
 * Rilascio del lock sulla funzione callback executionCallback
 */
void unlockLaser() {
    laserProcessing = false;
}//unlockLaser


/**
 * Rilascio del lock sulla funzione callback laserLineCallback
 */
void unlockLines() {
    linesProcessing = false;
}//unlockLines


/**
 * Funzione callback utilizzata per gestire il passaggio stretto all'andata
 * @param  scanPtr_  puntatore alla variabile contenente la scansione del laser
 */
void scanCallback(const sensor_msgs::LaserScanConstPtr& scanPtr_) {
    // Voglio che la distanza dal muro di destra e sinistra sia la stessa
    sensor_msgs::LaserScan scan = *scanPtr_;

    // Ricavo le distanze dalla lettura del laser
    double scanDX = navi->getLaserRightDistance(scan, utils->getParam("scan_right"));
    double scanSX = navi->getLaserLeftDistance(scan, utils->getParam("scan_left"));
    double scanFront = navi->getLaserFrontDistance(scan, utils->getParam("scan_front"));

    if(iamhere == 2 && (scanFront <= utils->getParam("iah2_stop_frontDistance") && scanSX < utils->getParam("iah2_stop_leftDistance"))) {
        // Sono nel passaggio stretto, eseguito quando Marrtino si deve fermare per andare nella piataforma di carico
        navi->mnav_stop();
        displayed = false;
        iamhere = 3;
        return;
    } else if(iamhere == 5 && scanDX > utils->getParam("iah5_exit_rightSpace")) {
        // Sto uscendo dal passaggio stretto, eseguito quando Marrtino si deve girare verso l'arena
        navi->mnav_stop();
        displayed = false;
        iamhere = 6;
        return;
    } else if(scanFront < utils->getParam("corridor_dist_frontObstacle")) { 
        // Troppo vicino ad un ostacolo davanti
        vel.linear.x = -utils->getParam("vel_recovery_linear_x");
        vel.linear.y = 0;
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;
        if(last != 0) {
            publish = true;
            std::cout << "STOP, arretra" << std::endl;
            last = 0;
        } else {
            publish = false;
        }//if else
    } else if(scanDX - scanSX > utils->getParam("EPSILON") && (scanDX < utils->getParam("corridor_dist_right") || scanSX < utils->getParam("corridor_dist_left"))) { 
        // Ho piu' spazio a destra che a sinistra, vado verso destra
        vel.linear.x = utils->getParam("vel_turn_linear_x");
        vel.angular.z = -utils->getParam("vel_turn_angular_z");
        if(last != 1) {
            publish = true;
            std::cout << "Vai verso destra" << std::endl;
            last = 1;
        } else {
            publish = false;
        }//if else
    } else if(scanSX - scanDX > utils->getParam("EPSILON") && (scanDX < utils->getParam("corridor_dist_right") || scanSX < utils->getParam("corridor_dist_left"))) { 
        // Ho piu' spazio a sinistra che a destra, vado verso sinistra
        vel.linear.x = utils->getParam("vel_turn_linear_x");
        vel.angular.z = utils->getParam("vel_turn_angular_z");
        if(last != 2) {
            publish = true;
            std::cout << "Vai verso sinistra" << std::endl;
            last = 2;
        } else {
            publish = false;
        }//if else
    } else { 
        // Sono ben posizionato al centro (con tolleranza), vado dritto
        vel.linear.x = utils->getParam("vel_std_linear_x");
        vel.angular.z = utils->getParam("vel_std_angular_z");
        if(last != 3) {
            publish = true;
            std::cout << "Vai dritto" << std::endl;
            last = 3;
        } else {
            publish = false;
        }//if else
    }//if else

    if(publish || !simulated) {
        // Si pubblica solo se l'ultima istruzione pubblicata e' diversa da quella attuale
        // Se si e' nel reale la pubblicazione avviene continuamente in ogni caso

        if(debug) {
            std::cout << "Scan DX = " << scanDX << std::endl;
            std::cout << "Scan Front = " << scanFront << std::endl;
            std::cout << "Scan SX = " << scanSX << std::endl;
            std::cout << "Vel X = " << vel.linear.x << std::endl;
            std::cout << "Vel Z = " << vel.angular.z << std::endl << std::endl;
        }//if

        ros::NodeHandle nh;
        ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
        ros::Time now = ros::Time::now();
        while(ros::ok()){
            pub.publish(vel);
            break;
        }//while
    }//if
    
}//scanCallback


/**
 * Funzione callback per riconoscere ed eventualmente agire rispetto ad una necessita' di recovery
 * @param  scanPtr_  puntatore alla variabile contenente la scansione del laser
 */
void recoveryCallback(const sensor_msgs::LaserScanConstPtr& scanPtr_) {
    sensor_msgs::LaserScan scan = *scanPtr_;

    // Ricavo le distanze dalla lettura del laser (legenda per A, B, C e D nella documentazione)

    // DESTRA
    double scanDXa = navi->getLaserRightDistanceA(scan);
    double scanDXb = navi->getLaserRightDistanceB(scan);
    double scanDXc = navi->getLaserRightDistanceC(scan);
    double scanDXd = navi->getLaserRightDistanceD(scan);

    // SINISTRA
    double scanSXa = navi->getLaserLeftDistanceA(scan);
    double scanSXb = navi->getLaserLeftDistanceB(scan);
    double scanSXc = navi->getLaserLeftDistanceC(scan);
    double scanSXd = navi->getLaserLeftDistanceD(scan);

    // FRONTALE
    double scanFront = navi->getLaserFrontDistance(scan);

    if(scanFront < utils->getParam("recovery_front_dist") || (navi->getObstacleRelativePosition(scan) == -1 && (scanDXa < utils->getParam("recovery_rightA_dist") || scanDXb < utils->getParam("recovery_rightB_dist") || scanDXd < utils->getParam("recovery_rightD_dist"))) || (navi->getObstacleRelativePosition(scan) == 1 && (scanSXa < utils->getParam("recovery_leftA_dist") || scanSXb < utils->getParam("recovery_leftB_dist") || scanSXd < utils->getParam("recovery_leftD_dist")))) {
        // Se un ostacolo e' troppo vicino ferma il planner ed esegui operazioni senza possibilita' che vengano bloccate da esiti di altre scansioni
        lock();

        // Ferma il planner indicando che il goal e' raggiunto (fittiziamente)
        navi->setRobotGoalPosition(navi->getCurrentRobotPosition().pose);
        falseGoalReached = true;

        // Ferma il robot
        navi->mnav_stop();

        // Aggiorna variabili di recovery
        ROS_INFO_STREAM("RECOVERY");
        consecutiveRecoveryCount++;
        consecutiveNoRecoveryCount = 0;
        recovery = true;

        navi->mnav_stop();

        if(debug) {
            std::cout << "Scan DXa = " << scanDXa << std::endl;
            std::cout << "Scan DXb = " << scanDXb << std::endl;
            std::cout << "Scan DXc = " << scanDXc << std::endl;
            std::cout << "Scan DXd = " << scanDXd << std::endl;
            std::cout << "Scan Front = " << scanFront << std::endl;
            std::cout << "Scan SXd = " << scanSXd << std::endl;
            std::cout << "Scan SXc = " << scanSXc << std::endl;
            std::cout << "Scan SXb = " << scanSXb << std::endl;
            std::cout << "Scan SXa = " << scanSXa << std::endl;
        }//if

        // Indietreggia
        navi->mnav_goStraight(1, -1);

        // Fine operazioni da eseguire in blocco
        unlock();
    } else if(recovery && ((navi->getObstacleRelativePosition(scan) == -1 && (scanDXa > utils->getParam("recovery_rightA_dist") || scanDXb > utils->getParam("recovery_rightB_dist"))) || (navi->getObstacleRelativePosition(scan) == 1 && (scanSXa > utils->getParam("recovery_leftA_dist") || scanSXb > utils->getParam("recovery_leftB_dist"))))) {
        // Si eseguono le altre operazioni di recovery se c'e' abbastanza spazio attorno
        lock();

        if(debug) {
            std::cout << "ConsecutiveRecoveryCount: " << consecutiveRecoveryCount << " ConsecutiveNoRecoveryCount: " << consecutiveNoRecoveryCount << std::endl;

            std::cout << "Scan DXa = " << scanDXa << std::endl;
            std::cout << "Scan DXb = " << scanDXb << std::endl;
            std::cout << "Scan DXc = " << scanDXc << std::endl;
            std::cout << "Scan DXd = " << scanDXd << std::endl;
            std::cout << "Scan Front = " << scanFront << std::endl;
            std::cout << "Scan SXd = " << scanSXd << std::endl;
            std::cout << "Scan SXc = " << scanSXc << std::endl;
            std::cout << "Scan SXb = " << scanSXb << std::endl;
            std::cout << "Scan SXa = " << scanSXa << std::endl;
        }//if

        // Operazioni uguali ma con parametri diversi in base a quante volte si e' tornati consecutivamente in recovery (+ volte => operazioni piu' aggressive)
        double degrees = 30;
        double speed = utils->getParam("vel_recovery_angular_z");
        int sign = 1;
        if(consecutiveRecoveryCount > utils->getParam("recovery_cons_third_thr")) {
            degrees = 180;
            speed = utils->getParam("vel_recovery_s4_angular_z");
            sign = 1;
            consecutiveRecoveryCount = 0;
        } else if(consecutiveRecoveryCount > utils->getParam("recovery_cons_second_thr")) {
            degrees = 90;
            speed = utils->getParam("vel_recovery_s3_angular_z");
            sign = 1;
        } else if(consecutiveRecoveryCount > utils->getParam("recovery_cons_first_thr")) {
            degrees = 40;
            speed = utils->getParam("vel_recovery_s2_angular_z");
            sign = 1;
        } else {
            degrees = 25;
            speed = utils->getParam("vel_recovery_angular_z");
            sign = 1;
        }//if else

        // Ci si gira dalla parte opposta all'ostacolo piu' vicino
        navi->mnav_turnAngleDegrees(degrees, sign * (-1) * navi->getObstacleRelativePosition(scan), speed);

        // Si avanza leggermente per non ritrovare l'ostacolo
        navi->mnav_goStraight(1, 1, utils->getParam("vel_recovery_linear_x"));

        // Tentativo di uscita dalla recovery, eventualmente si rientrera' subito
        recovery = false;
        ROS_INFO_STREAM("EXIT RECOVERY");

        // Bisogna riavviare il planner
        planSent0 = false;
        planSent7 = false;
        planSent9 = false;
        unlock();
    } else if(!recovery) {
        // Se non si e' in recovery da un po' si resetta la variabile di stato (non subito per evitare falsi positivi)
        if(consecutiveNoRecoveryCount++ >= utils->getParam("recovery_nocons_thr")) {
            consecutiveRecoveryCount = 0;
        }//if
    }//if else
}//recoveryCallback


/**
 * Funzione callback principale, scandisce l'andamento del programma con la ricezione delle letture dal laser
 * @param  scanPtr_  puntatore alla variabile contenente la scansione del laser
 */
void executionCallback(const sensor_msgs::LaserScanConstPtr& scanPtr_) {
    // Se mutex e' true non c'e' nulla da fare, lettura cestinata
    // Vuol dire che e' in esecuzione una serie di istruzioni non interrompibili
    // E' inutile mettere in coda i messaggi perche' hanno validita' solo nell'istante della lettura
    if(processing || linesProcessing)
        return;

    sensor_msgs::LaserScan scan = *scanPtr_;
    //navi->getRearLaserData(scan);
    if(iamhere == 0) {
        // Devo partire dalla posizione iniziale ed arrivare vicino al passaggio stretto
        if(!displayed) {
            ROS_INFO_STREAM("iamhere 0");
            displayed = true;
        }//if
        if(recovery) {
            // Se sono gia' in recovery passa il controllo all'altra routine
            recoveryCallback(scanPtr_);
        } else if(!turned0) {
            // Inizialmente il robot si deve girare
            if(forceRotation) {
                if(is360laser) {
                    sensor_msgs::LaserScan leftScan;
                    leftScan = navi->getLeftLaserData(scan);
                    double sum = 0;
                    int numScan = leftScan.ranges.size();
                    for(int i = 0;i < numScan;i++) {
                        sum += leftScan.ranges[i];
                    }//for
                    double leftDistance = sum / (double) numScan;

                    sensor_msgs::LaserScan rearScan;
                    rearScan = navi->getRearLaserData(scan);
                    sum = 0;
                    numScan = rearScan.ranges.size();
                    for(int i = 0;i < numScan;i++) {
                        sum += rearScan.ranges[i];
                    }//for
                    double rearDistance = sum / (double) numScan;
                    if(leftDistance > utils->getParam("iah0_left_distance") && rearDistance > utils->getParam("iah0_rear_distance")) {
                        navi->mnav_turnAngleDegrees(90, 1, utils->getParam("vel_inplace_rot_angular_z"), true);
                    }//if
                } else {
                    navi->mnav_turnAngleDegrees(90, 1, utils->getParam("vel_inplace_rot_angular_z"), true);
                }//if else
            }//if  
            turned0 = true;
        } else if(!planSent0) {
            // Si manda al planner la posizione del goal da raggiungere (prima volta o dopo ritorno da recovery)
            // Goal leggermente diversi tra caso simulato e reale per piccole differenze nella mappa
            ROS_INFO_STREAM("Invio del goal!");
            /// TO-DO
            if(simulated) {
                navi->setRobotGoalPosition(utils->createPose(-0.017, -1.797, 0, 0, 0, 0.001, 1.0), true);
            } else {
                navi->setRobotGoalPosition(utils->createPose(0.142, -1.547, 0, 0, 0, 0.006, 1.0), true);
            }//if else
            planSent0 = true;
        } else {
            // Se non sono in recovery e sto navigando controllo che non ci siano le condizioni per andare in recovery
            recoveryCallback(scanPtr_);
            // Leggo lo stato inviato dal planner
            ros::NodeHandle nh;
            actionlib_msgs::GoalStatusArrayConstPtr status_check_ = ros::topic::waitForMessage<actionlib_msgs::GoalStatusArray>("/marrtino/move_base/status", nh);
            actionlib_msgs::GoalStatus status = status_check_->status_list[status_check_->status_list.size() - 1];
            int status_id = status.status;
            if(status_id == 1) {
                // Navigazione in corso
                ROS_INFO_STREAM("Navigazione in corso");
                falseGoalReached = false;
            } else if(status_id == 3 && !falseGoalReached) {
                // E' stato raggiunto il goal (per davvero, non quello fittizio), si passa alla fase successiva
                ROS_INFO_STREAM("Goal Raggiunto!");
                iamhere = 1;
                displayed = false;
            } else if(status_id == 4) {
                // Il planner indica una situazione di errore bloccante, si riprova a inviare il goal e si mette la recovery in condizione di agire piu' aggressivamente
                if(simulated) {
                    navi->setRobotGoalPosition(utils->createPose(-0.017, -1.797, 0, 0, 0, 0.001, 1.0), true);
                } else {
                    navi->setRobotGoalPosition(utils->createPose(0.142, -1.547, 0, 0, 0, 0.006, 1.0), true);
                }//if else
                planSent0 = true;
                consecutiveRecoveryCount = 3;
                recovery = true;
            }//if else
        }//if else
    } else if(iamhere == 1) {
        // Devo far imboccare al robot il passaggio stretto
        if(!displayed) {
            ROS_INFO_STREAM("iamhere 1");
            displayed = true;
        }//if
        if(!wellpositioned1) {
            // Posiziono Marrtino parallelo al muro
            if(debug) {
                std::cout << fabs(navi->getLaserRightDistanceD(scan) - navi->getLaserLeftDistanceD(scan)) << " - " << navi->getLaserRightDistanceD(scan) << " - " << navi->getLaserLeftDistanceD(scan) << std::endl;
            }//if
            if(!chosen1) {
                // Operazione demandata a laserLineCallback
            }//if
        } else if(!approached1) {
            // Marrtino si avvicina al muro
            if(navi->getLaserFrontDistance(scan) < utils->getParam("iah1_stop_dist_front") || navi->getLaserLeftDistanceD(scan) < utils->getParam("iah1_stop_dist_leftD") || navi->getLaserRightDistanceD(scan) < utils->getParam("iah1_stop_dist_rightD")) {
                // Si e' avvicinato, si ferma
                lock();
                vel.linear.x = 0;
                vel.angular.z = 0;
                navi->setRobotVelocity(vel, 1, true);
                approached1 = true;
                ROS_INFO_STREAM("Avvicinato!");
                unlock();
            } else {
                if(navi->getLaserLeftDistanceD(scan) - navi->getLaserRightDistanceD(scan) > utils->getParam("EPSILON_FINE")) {
                    // Ho piu' spazio a destra che a sinistra, vado verso destra
                    vel.linear.x = 0.1;
                    vel.angular.z = -0.1;
                    navi->setRobotVelocity(vel, 1, true);
                } else if(navi->getLaserRightDistanceD(scan) - navi->getLaserLeftDistanceD(scan) > utils->getParam("EPSILON_FINE")) { 
                    // Ho piu' spazio a sinistra che a destra, vado verso sinistra
                    vel.linear.x = 0.1;
                    vel.angular.z = 0.1;
                    navi->setRobotVelocity(vel, 1, true);
                } else { 
                    // Vado dritto
                    vel.angular.z = 0;
                    vel.linear.x = utils->getParam("vel_std_linear_x");
                    navi->setRobotVelocity(vel, 1, true);
                }//if else
            }//if else            
        } else if(!turned1) {
            // Si ruota il robot in modo da posizionarsi davanti all'ingresso del passaggio stretto piu' precisamente possibile
            // Le correzioni verranno effettuate dopo
            navi->mnav_turnAngleDegrees(90, 1, utils->getParam("vel_inplace_rot_angular_z"));
            lock();
            turned1 = true;
            ROS_INFO_STREAM("Ruotato!");
            unlock();
        } else if(!wellpositioned12) {
            // Entra nell'imbocco del passaggio stretto piu' centrato possibile
            if(navi->getLaserLeftDistanceB(scan) < utils->getParam("iah1_corr_entrance_dist_leftB")) {
                // Si e' all'imbocco del passaggio stretto
                wellpositioned12 = true;
                navi->mnav_stop();
            }//if 
        } else {
            lock();
            ROS_INFO_STREAM("Davanti al corridoio!");
            // Si va al passaggio successivo
            iamhere = 2;
            insideCorridor2 = true;
            displayed = false;
            unlock();
        }//if else
    } else if(iamhere == 2) {
        // Si e' entrati nel passaggio stretto
        if(!displayed) {
            ROS_INFO_STREAM("iamhere 2");
            displayed = true;
        }//if
        // Gestione della navigazione nel passaggio stretto demandata ad una callback apposita
        if(utils->getParam("config_use_lines") == 0)
            scanCallback(scanPtr_);
    } else if(iamhere == 3) {
        // Si e' arrivati nel punto in cui bisogna ruotare per avvicinarsi alla piattaforma di carico
        if(!displayed) {
            ROS_INFO_STREAM("iamhere 3");
            displayed = true;
        }//if
        if(!turned3) {
            // Operazione demandata a laserLineCallback
        } else if(!docked3) {
            if(navi->getLaserFrontDistance(scan) < utils->getParam("iah3_stop_dist_front") || navi->getLaserLeftDistanceD(scan) < utils->getParam("iah3_stop_dist_leftD") || navi->getLaserRightDistanceD(scan) < utils->getParam("iah3_stop_dist_rightD")) {
                // Avvicinamento eseguito, si ferma davanti al muro per attendere carico degli oggetti
                lock();
                vel.linear.x = 0;
                vel.angular.z = 0;
                navi->setRobotVelocity(vel, 1, true);
                docked3 = true;
                ROS_INFO_STREAM("Arrivato!");
                // Passaggio al passo successivo
                // Comunicazione del proprio stato "Arrivato alla stazione di carico" al server
                ros::NodeHandle nh;
                ros::ServiceClient clientNavigationToFSM = nh.serviceClient<g06_fsm::NavigationToFSM>("/NavigationToFSM");
                g06_fsm::NavigationToFSM serviceN2FSM;
                serviceN2FSM.request.iamhere = iamhere;
                geometry_msgs::PoseStamped currPose;
                currPose.header.frame_id = "world";
                currPose.header.stamp = ros::Time::now();
                // Posizione del robot ottenuta con l'odometria invece che con AMCL
                currPose.pose = navi->getCurrentRobotPosition(true).pose;
                serviceN2FSM.request.robotPose = currPose;
                if(clientNavigationToFSM.call(serviceN2FSM)) {
                    if(serviceN2FSM.response.ack == 1 && serviceN2FSM.response.gonext == 1) {
                        ROS_INFO_STREAM("Navigazione ok e continua! NavigationToFSM");
                    } else {

                    }//if else
                } else {
                    ROS_INFO_STREAM("Errore! NavigationToFSM");
                }//if else
                iamhere = 4;
                displayed = false;
                unlock();
            } else {
                // Avvicinamento al muro in modo più dritto possibile
                if(navi->getLaserLeftDistanceB(scan) - navi->getLaserRightDistanceB(scan) > utils->getParam("EPSILON_FINE")) { 
                    // Ho piu' spazio a destra che a sinistra, vado verso destra
                    vel.linear.x = 0.1;
                    vel.angular.z = -0.1;
                    navi->setRobotVelocity(vel, 1, true);
                } else if(navi->getLaserRightDistanceB(scan) - navi->getLaserLeftDistanceB(scan) > utils->getParam("EPSILON_FINE")) { 
                    // Ho piu' spazio a sinistra che a destra, vado verso sinistra
                    vel.linear.x = 0.1;
                    vel.angular.z = 0.1;
                    navi->setRobotVelocity(vel, 1, true);
                } else { 
                    // Vado dritto
                    vel.angular.z = 0;
                    vel.linear.x = 0.1;
                    navi->setRobotVelocity(vel, 1, true);
                }//if else
            }//if else
        }//if else
    } else if(iamhere == 4) {
        // Marrtino si può spostare dalla piattaforma di carico
        if(!displayed) {
            ROS_INFO_STREAM("iamhere 4");
            displayed = true;
        }//if
        if(!undocked4) {
            if(navi->getLaserFrontDistance(scan) < utils->getParam("iah4_stop_dist_front") || navi->getLaserLeftDistanceD(scan) < utils->getParam("iah4_stop_dist_leftD") || navi->getLaserRightDistanceD(scan) < utils->getParam("iah4_stop_dist_rightD")) {
                // Si allontana dal muro
                vel.linear.x = -0.1;
                vel.angular.z = 0;
                navi->setRobotVelocity(vel, 1, true);
            } else {
                // Si ferma per iniziare la rotazione
                lock();
                vel.linear.x = 0;
                vel.angular.z = 0;
                navi->setRobotVelocity(vel, 1, true);
                undocked4 = true;
                ROS_INFO_STREAM("Allontanato! Ora si deve ruotare");
                unlock();
            }//if else
        } else if(!turned4) {
            // Operazione demandata a laserLineCallback
        }//if else
    } else if(iamhere == 5) {
        // Entra e naviga nel corridoio stretto
        if(!displayed) {
            ROS_INFO_STREAM("iamhere 5");
            displayed = true;
        }//if
        last = -1;
        // Gestione della navigazione nel passaggio stretto demandata a laserLineCallback
        double scanDX = navi->getLaserRightDistance(scan, utils->getParam("scan_right"));
        if(scanDX > utils->getParam("iah5_exit_rightSpace")) {
            // Sto uscendo dal passaggio stretto, eseguito quando Marrtino si deve girare verso l'arena
            navi->mnav_stop();
            displayed = false;
            iamhere = 6;
            return;
        }//if
    } else if(iamhere == 6) {
        // Si e' usciti dal passaggio stretto, bisogna posizionarsi davanti all'arena
        if(!displayed) {
            ROS_INFO_STREAM("iamhere 6");
            displayed = true;
        }//if
        if(!positioned6) {
            // Quando riconosce di essere all'uscita del corridoio va dritto per 1 secondo
            navi->mnav_goStraight(1, 1, 0.2);
            lock();
            positioned6 = true;
            ROS_INFO_STREAM("Uscito dal passaggio stretto! Ora deve ruotare");
            unlock();
        } else if(!turned6) {
            // Successivamente va dritto con una leggera curvatura
            lock();
            navi->mnav_goStraightWithCurve(3, 1, utils->getParam("iah6_vel_curve_linear_x"), utils->getParam("iah6_vel_curve_angular_z"));
            vel.linear.x = 0;
            vel.angular.z = 0;
            navi->setRobotVelocity(vel, 1, true);
            // Ora si ruota in modo da mettersi esattamente contro l'arena
            navi->mnav_turnAngleDegrees(45, -1, 0.8);
            vel.linear.x = 0;
            vel.angular.z = 0;
            navi->setRobotVelocity(vel, 1, true);
            turned6 = true;
            ROS_INFO_STREAM("Ruotato! Ora va nell'arena");
            // Si passa alla fase successiva
            iamhere = 7;
            displayed = false;
            unlock();
        }//if else
    } else if(iamhere == 7) {
        // Marrtino deve navigare dall'uscita del passaggio stretto alla piattaforma di scarico evitando gli ostacoli
        // ed eventualmente ricorrendo alla procedura di recovery
        if(!displayed) {
            ROS_INFO_STREAM("iamhere 7");
            displayed = true;
        }//if
        if(debug) {
            std::cout << "Recovery: " << recovery << " FalseGoalReached: " << falseGoalReached << std::endl;
        }//if
        if(recovery) {
            // Se sono gia' in recovery passa il controllo all'altra routine
            recoveryCallback(scanPtr_);
        } else if(!planSent7) {
            // Si manda al planner la posizione del goal da raggiungere (prima volta o dopo ritorno da recovery)
        	ROS_INFO_STREAM("Invio del goal!");
            navi->setRobotGoalPosition(utils->createPose(-1.290, -0.356, 0, 0, 0, 1, 0), true);
            planSent7 = true;
        } else {
            // Se non sono in recovery e sto navigando controllo che non ci siano le condizioni per andare in recovery
            recoveryCallback(scanPtr_);
            // Leggo lo stato inviato dal planner
            ros::NodeHandle nh;
            actionlib_msgs::GoalStatusArrayConstPtr status_check_ = ros::topic::waitForMessage<actionlib_msgs::GoalStatusArray>("/marrtino/move_base/status", nh);
            actionlib_msgs::GoalStatus status = status_check_->status_list[status_check_->status_list.size() - 1];
            int status_id = status.status;
            if(debug) {
                std::cout << "Status è " << status_id << std::endl;
            }//if
            if(status_id == 1) {
                // Navigazione in corso
                ROS_INFO_STREAM("Navigazione in corso");
                falseGoalReached = false;
            } else if(status_id == 3 && !falseGoalReached) {
                // E' stato raggiunto il goal (per davvero, non quello fittizio), si passa alla fase successiva
                ROS_INFO_STREAM("Goal Raggiunto!");
                iamhere = 8;
                displayed = false;
            } else if(status_id == 4) {
                // Il planner indica una situazione di errore bloccante, si riprova a inviare il goal e si mette la recovery in condizione di agire piu' aggressivamente
                navi->setRobotGoalPosition(utils->createPose(-1.290, -0.356, 0, 0, 0, 1, 0), true);
                planSent7 = true;
                consecutiveRecoveryCount = 3;
                recovery = true;
            }//if else
        }//if else
    } else if(iamhere == 8) {
        // Marrtino e' arrivato nei pressi della piattaforma di scarico
        if(!displayed) {
            ROS_INFO_STREAM("iamhere 8");
            displayed = true;
        }//if
        if(!wellpositioned8) {
            // Operazione demandata a laserLineCallback
        } else if(!docked8) {
            // Marrtino si avvicina al muro
            if(navi->getLaserFrontDistance(scan) < utils->getParam("iah8_stop_dist_front") || navi->getLaserLeftDistanceD(scan) < utils->getParam("iah8_stop_dist_leftD") || navi->getLaserRightDistanceD(scan) < utils->getParam("iah8_stop_dist_rightD")) {
                lock();
                vel.linear.x = 0;
                vel.angular.z = 0;
                navi->setRobotVelocity(vel, 1, true);
                docked8 = true;
                ROS_INFO_STREAM("Arrivato!");
                // Comunicazione del proprio stato "Arrivato alla stazione di scarico" al server
                ros::NodeHandle nh;
                ros::ServiceClient clientNavigationToFSM = nh.serviceClient<g06_fsm::NavigationToFSM>("/NavigationToFSM");
                g06_fsm::NavigationToFSM serviceN2FSM;
                serviceN2FSM.request.iamhere = iamhere;
                geometry_msgs::PoseStamped currPose;
                currPose.header.frame_id = "world";
                currPose.header.stamp = ros::Time::now();
                currPose.pose = navi->getCurrentRobotPosition().pose;
                serviceN2FSM.request.robotPose = currPose;
                if(clientNavigationToFSM.call(serviceN2FSM)) {
                    if(serviceN2FSM.response.ack == 1) {
                        ROS_INFO_STREAM("Navigazione ok! NavigationToFSM");
                        if(serviceN2FSM.response.gonext == 1) {
                            iamhere = 9;
                            ROS_INFO_STREAM("Continua navigazione! NavigationToFSM");
                        } else {
                            // Fine della navigazione
                            ROS_INFO_STREAM("Navigazione terminata! NavigationToFSM");
                            ros::shutdown();
                        }//if else
                    } else {
                        ROS_INFO_STREAM("Errore! NavigationToFSM");
                        ros::shutdown();
                    }//if else
                } else {
                    ROS_INFO_STREAM("Errore! NavigationToFSM");
                }//if else
                iamhere = 9;
                displayed = false;
                unlock();
            } else {
                if(navi->getLaserLeftDistanceD(scan) - navi->getLaserRightDistanceD(scan) > utils->getParam("EPSILON_FINE")) { 
                    // Ho piu' spazio a destra che a sinistra, vado verso destra
                    vel.linear.x = 0.1;
                    vel.angular.z = -0.1;
                    navi->setRobotVelocity(vel, 1, true);
                } else if(navi->getLaserRightDistanceD(scan) - navi->getLaserLeftDistanceD(scan) > utils->getParam("EPSILON_FINE")) { 
                    // Ho piu' spazio a sinistra che a destra, vado verso sinistra
                    vel.linear.x = 0.1;
                    vel.angular.z = 0.1;
                    navi->setRobotVelocity(vel, 1, true);
                } else { 
                    // Vado dritto
                    vel.angular.z = 0;
                    vel.linear.x = 0.1;
                    navi->setRobotVelocity(vel, 1, true);
                }//if else
            }//if else
        }//if else
    } else if(iamhere == 9) {
        // Undocking di Marrtino dall'area di scarico e la navigazione verso il passaggio stretto
        if(!displayed) {
            ROS_INFO_STREAM("iamhere 9");
            displayed = true;
            ROS_INFO_STREAM("Attesa scarico oggetti");
        }//if
        if(!undocked9) {
            if(navi->getLaserFrontDistance(scan) < utils->getParam("iah9_stop_dist_front") || navi->getLaserLeftDistanceD(scan) < utils->getParam("iah9_stop_dist_leftD") || navi->getLaserRightDistanceD(scan) < utils->getParam("iah9_stop_dist_rightD")) {
                vel.linear.x = -0.1;
                vel.angular.z = 0;
                navi->setRobotVelocity(vel, 1, true);
            } else {
                lock();
                vel.linear.x = 0;
                vel.angular.z = 0;
                navi->setRobotVelocity(vel, 1, true);
                undocked9 = true;
                ROS_INFO_STREAM("Allontanato! Ora si deve ruotare");
                unlock();
            }//if else
        } else if(!rotated9) {
            navi->mnav_turnAngleDegrees(180, 1, 1);
            lock();
            vel.linear.x = 0;
            vel.angular.z = 0;
            navi->setRobotVelocity(vel, 1, true);
            rotated9 = true;
            ROS_INFO_STREAM("Ruotato! Fine");
            displayed = false;
            unlock();
        } else {
            if(recovery) {
                recoveryCallback(scanPtr_);
            } else if(!planSent9) {
                ROS_INFO_STREAM("Invio del piano!");
                if(simulated) {
                    navi->setRobotGoalPosition(utils->createPose(-0.017, -1.797, 0, 0, 0, 0.001, 1.0), true);
                } else {
                    navi->setRobotGoalPosition(utils->createPose(0.142, -1.547, 0, 0, 0, 0.006, 1.0), true);
                }//if else
                planSent9 = true;
            } else {
                recoveryCallback(scanPtr_);
                ros::NodeHandle nh;
                actionlib_msgs::GoalStatusArrayConstPtr status_check_ = ros::topic::waitForMessage<actionlib_msgs::GoalStatusArray>("/marrtino/move_base/status", nh);
                actionlib_msgs::GoalStatus status = status_check_->status_list[status_check_->status_list.size() - 1];
                int status_id = status.status;
                if(debug) {
                    std::cout << "Status è " << status_id << std::endl;
                }//if
                if(status_id == 1) {
                    ROS_INFO_STREAM("Navigazione in corso");
                    falseGoalReached = false;
                } else if(status_id == 3 && !falseGoalReached) {
                    ROS_INFO_STREAM("Goal Raggiunto!");
                    lock();
                    resetStatus();
                    iamhere = 1;
                    unlock();
                    displayed = false;
                } else if(status_id == 4) {
                    if(simulated) {
                        navi->setRobotGoalPosition(utils->createPose(-0.017, -1.797, 0, 0, 0, 0.001, 1.0), true);
                    } else {
                        navi->setRobotGoalPosition(utils->createPose(0.142, -1.547, 0, 0, 0, 0.006, 1.0), true);
                    }//if else
                    planSent9 = true;
                    consecutiveRecoveryCount = 3;
                    recovery = true;
                }//if else
            }//if else
        }//if else
    } else {
        ROS_ERROR_STREAM("Situazione impossibile da verificarsi! Non e' da nessuna parte");
    }//if else
    unlock();
}//executionCallback


/**
 * Funzione callback per la rilevazione delle linee dalla lettura del laser
 * @param  linesListPtr_  puntatore alla variabile contenente la lista delle linee rilevate
 */
void laserLineCallback(const laser_line_extraction::LineSegmentListConstPtr& linesListPtr_) {
    // Se mutex e' true non c'e' nulla da fare, lettura cestinata
    // Vuol dire che e' in esecuzione una serie di istruzioni non interrompibili
    // E' inutile mettere in coda i messaggi perche' hanno validita' solo nell'istante della lettura
    if(processing || laserProcessing)
        return;

    laser_line_extraction::LineSegmentList linesList = *linesListPtr_;
    std::vector<laser_line_extraction::LineSegment> lines = linesList.line_segments;
    int numLines = linesList.line_segments.size();

    if(numLines > 0) {
        if(iamhere == 1) {
            if(!wellpositioned1) {
                if(debug) {
                    std::cout << "Linee: " << numLines << std::endl;
                    for(int i = 0;i < numLines;i++) {
                        std::cout << i << ": " << linesList.line_segments[i].angle << " " << linesList.line_segments[i].radius << std::endl;
                    }//for
                }//if
                // Devo identificare il muro a cui Marrtino sara' parallelo (e' quello a cui sono gia' piu' parallelo)
                if(!chosen1) {
                    double minAngle = navi->getFrontLine(linesList).angle;
                    rot_sign1 = minAngle > 0 ? 1 : -1;
                    ROS_INFO_STREAM(minAngle);
                    lockLines();
                    chosen1 = true;
                    if(debug) {
                        std::cout << "Rotazione in corso: " << minAngle << std::endl;
                    }//if
                    navi->mnav_turnAngleRadians(fabs(minAngle), rot_sign1, utils->getParam("vel_inplace_rot_angular_z"));
                    if(debug) {
                        std::cout << "Rotazione effettuata" << std::endl;
                    }//if   
                    lock();
                    wellpositioned1 = true;
                    ROS_INFO_STREAM("Parallelo al muro! Ora deve entrare nel passaggio stretto");
                    unlock();
                    unlockLines();
                }//if else
            } else if(!turned1) {
                // Lasciato a executionCallback
            } else if(!wellpositioned12) {
                // Wall follower rispetto al muro destro (non considera la distanza dal muro, solo l'inclinazione)
                // Trova i muri a destra e prende il piu' vicino
                laser_line_extraction::LineSegment rightWall = navi->getRightLine(linesList);

                if(debug) {
                    std::cout << "Muro destro: " << rightWall.angle << " " << rightWall.radius << std::endl;
                }//if

                double errorAngle = rightWall.angle + (M_PI / 2);
                if(debug) {
                    std::cout << "errorAngle: " << errorAngle << std::endl;
                }//if
                vel.linear.x = 0.1;
                vel.angular.z = errorAngle >= 0 ? std::min(std::max(errorAngle, 0.1), 0.3) : std::max(std::min(errorAngle, -0.1), -0.3);

                if(debug) {
                    std::cout << "Scan DX = " << rightWall.radius << std::endl;
                    std::cout << "Vel X = " << vel.linear.x << std::endl;
                    std::cout << "Vel Z = " << vel.angular.z << std::endl << std::endl;
                }//if

                ros::NodeHandle nh;
                ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
                ros::Time now = ros::Time::now();
                while(ros::ok()){
                    while(pub.getNumSubscribers() == 0) {
                        usleep(1);
                    }//while
                    pub.publish(vel);
                    break;
                }//while

                // Il passaggio ad iamhere2 e' gestito da executionCallback

            }//if else
        } else if(iamhere == 2) {
            if(utils->getParam("config_use_lines") == 1) {
                // Sostituisce scanCallback
                // Deve mantenere piu' o meno 90 gradi a destra e a sinistra
                // Cerca i 3 muri: sinistro, destro, frontale
                // Quando la prima parte del destro termina, segue solo il sinistro

                laser_line_extraction::LineSegment leftWall = navi->getLeftLine(linesList);
                laser_line_extraction::LineSegment rightWall = navi->getRightLine(linesList);
                laser_line_extraction::LineSegment frontWall = navi->getFrontLine(linesList);

                if(debug) {
                    std::cout << "Muro sinistro: " << leftWall.angle << " " << leftWall.radius << std::endl;
                    std::cout << "Muro frontale: " << frontWall.angle << " " << frontWall.radius << std::endl;
                    std::cout << "Muro destro: " << rightWall.angle << " " << rightWall.radius << std::endl;
                }//if

                if(insideCorridor2) {
                    // Sono nel corridoio, si usa il wall following prendendo come riferimento il muro destro
                    double errorAngle = rightWall.angle + (M_PI / 2);
                    double errorDistanceRight = utils->getParam("iah2_rightwall_dist") - rightWall.radius;
                    if(debug) {
                        std::cout << "errorAngle: " << errorAngle << std::endl;
                        std::cout << "erroreDistanceRight: " << errorDistanceRight << std::endl;
                    }//if
                    vel.linear.x = fabs(errorAngle + (errorDistanceRight * 2)) > 0.1 ? 0.1 : 0.3;
                    vel.angular.z = errorAngle + (errorDistanceRight * 2) >= 0 ? std::min(errorAngle + (errorDistanceRight * 2), 0.3) : std::max(errorAngle + (errorDistanceRight * 2), -0.3);
                    publish = true;
                    last = 1;

                    // L'uscita dal corridoio e' gestita da executionCallback
                } else {
                    // Verso la base di carico
                    // Si mantiene la distanza solo dal muro sinistro
                    if(debug) {
                        std::cout << "insideCorridor2: " << insideCorridor2 << std::endl;
                    }//if
                    if(frontWall.radius <= utils->getParam("iah2_stop_frontDistance")) {
                        // Sono nel passaggio stretto, eseguito quando Marrtino si deve fermare per andare nella piataforma di carico
                        navi->mnav_stop();
                        displayed = false;
                        iamhere = 3;
                        return;
                    } else if(frontWall.radius < utils->getParam("corridor_dist_frontObstacle")) { 
                        // Troppo vicino ad un ostacolo davanti
                        vel.linear.x = -utils->getParam("vel_recovery_linear_x");
                        vel.linear.y = 0;
                        vel.linear.z = 0;
                        vel.angular.x = 0;
                        vel.angular.y = 0;
                        vel.angular.z = 0;
                        if(last != 0) {
                            publish = true;
                            std::cout << "STOP, arretra" << std::endl;
                            last = 0;
                        } else {
                            publish = false;
                        }//if else
                    } else {
                        // Sono fuori dal corridoio, si usa il wall following prendendo come riferimento il muro sinistro
                        double errorAngle = leftWall.angle - (M_PI / 2);
                        double errorDistanceLeft = leftWall.radius - utils->getParam("iah2_leftwall_dist");
                        if(debug) {
                            std::cout << "errorAngle: " << errorAngle << std::endl;
                            std::cout << "erroreDistanceLeft: " << errorDistanceLeft << std::endl;
                        }//if
                        vel.linear.x = fabs(errorAngle + (errorDistanceLeft * 2)) > 0.1 ? 0.1 : 0.3;
                        vel.angular.z = errorAngle + (errorDistanceLeft * 2) >= 0 ? std::min(errorAngle + (errorDistanceLeft * 2), 0.3) : std::max(errorAngle + (errorDistanceLeft * 2), -0.3);
                        publish = true;
                        last = 1;
                    }//if else
                }//if else            

                if(publish || !simulated) {
                    // Si pubblica solo se l'ultima istruzione pubblicata e' diversa da quella attuale
                    // Se si e' nel reale la pubblicazione avviene continuamente in ogni caso

                    if(debug) {
                        std::cout << "Scan DX = " << rightWall.radius << std::endl;
                        std::cout << "Scan Front = " << frontWall.radius << std::endl;
                        std::cout << "Scan SX = " << leftWall.radius << std::endl;
                        std::cout << "Vel X = " << vel.linear.x << std::endl;
                        std::cout << "Vel Z = " << vel.angular.z << std::endl << std::endl;
                    }//if

                    ros::NodeHandle nh;
                    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
                    ros::Time now = ros::Time::now();
                    while(ros::ok()){
                        while(pub.getNumSubscribers() == 0) {
                            usleep(1);
                        }//while
                        pub.publish(vel);
                        break;
                    }//while
                }//if
            }//if

        } else if(iamhere == 3) {
            if(!turned3) {
                if(debug) {
                    std::cout << "Linee: " << numLines << std::endl;
                    for(int i = 0;i < numLines;i++) {
                        std::cout << i << ": " << linesList.line_segments[i].angle << " " << linesList.line_segments[i].radius << std::endl;
                    }//for
                }//if

                // Trova il muro frontale rispetto a Marrtino
                laser_line_extraction::LineSegment frontWall = navi->getFrontLine(linesList);

                // Ruotero' verso sinistra di 90 gradi + l'orientazione attuale nei confronti del muro frontale
                lock();
                if(debug) {
                    std::cout << "Ruota di: " << (M_PI / 2) + frontWall.angle << std::endl;
                }//if
                navi->mnav_turnAngleRadians((M_PI / 2) + frontWall.angle, 1, utils->getParam("vel_inplace_rot_angular_z"));

                // Parallelo al muro, ferma il robot
                vel.linear.x = 0;
                vel.angular.z = 0;
                navi->setRobotVelocity(vel, 1, true);
                turned3 = true;
                ROS_INFO_STREAM("Ruotato! Ora si deve avvicinare");
                unlock();
            }//if
            // Approaching lasciato a executionCallback
        } else if(iamhere == 4) {
            // Marrtino si puo' spostare dalla piattaforma di carico
            if(!undocked4) {
                // Attende undock
            } else if(!turned4) {
                // Controlla di quanto deve ruotare
                laser_line_extraction::LineSegment frontWall = navi->getFrontLine(linesList);
                if(frontWall.radius == 0) {
                    // Muro non trovato, usa il comportamento generico
                    navi->mnav_turnAngleDegrees(100, 1, utils->getParam("vel_inplace_rot_angular_z"));
                } else {
                    navi->mnav_turnAngleRadians(frontWall.angle + (M_PI / 2), 1, utils->getParam("vel_inplace_rot_angular_z"));
                }//if else
                lock();
                vel.linear.x = 0;
                vel.angular.z = 0;
                navi->setRobotVelocity(vel, 1, true);
                turned4 = true;
                ROS_INFO_STREAM("Ruotato! Ora va nel passaggio stretto");
                // Si va al passaggio successivo
                iamhere = 5;
                displayed = false;
                unlock();
            }//if else
        } else if(iamhere == 5) {
            // Sono all'imbocco oppure all'interno del corridoio, si usa il wall following prendendo come riferimento il muro destro
            laser_line_extraction::LineSegment rightWall = navi->getRightLine(linesList);

            if(rightWall.angle == 0) {
                // Muro non piu' rilevato
                // Si sta uscendo dal corridoio
                navi->mnav_stop();
                displayed = false;
                iamhere = 6;
                return;
            }//if

            if(debug) {
                std::cout << "Muro destro: " << rightWall.angle << " " << rightWall.radius << std::endl;
            }//if
            double errorAngle = rightWall.angle + (M_PI / 2);
            double errorDistanceRight = utils->getParam("iah2_rightwall_dist") - rightWall.radius;
            if(debug) {
                std::cout << "errorAngle: " << errorAngle << std::endl;
                std::cout << "erroreDistanceRight: " << errorDistanceRight << std::endl;
            }//if
            vel.linear.x = fabs(errorAngle + (errorDistanceRight * 2)) > 0.1 ? 0.1 : 0.3;
            vel.angular.z = errorAngle + (errorDistanceRight * 2) >= 0 ? std::min(errorAngle + (errorDistanceRight * 2), 0.3) : std::max(errorAngle + (errorDistanceRight * 2), -0.3);
            publish = true;
            last = 1;

            if(publish || !simulated) {
                // Si pubblica solo se l'ultima istruzione pubblicata e' diversa da quella attuale
                // Se si e' nel reale la pubblicazione avviene continuamente in ogni caso

                if(debug) {
                    std::cout << "Scan DX = " << rightWall.radius << std::endl;
                    std::cout << "Vel X = " << vel.linear.x << std::endl;
                    std::cout << "Vel Z = " << vel.angular.z << std::endl << std::endl;
                }//if

                ros::NodeHandle nh;
                ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
                ros::Time now = ros::Time::now();
                while(ros::ok()){
                    while(pub.getNumSubscribers() == 0) {
                        usleep(1);
                    }//while
                    pub.publish(vel);
                    break;
                }//while
            }//if

            // L'uscita dal corridoio e' gestita da executionCallback se non avviene a causa della non rilevazione del muro destro
        } else if(iamhere == 8) {
            if(!wellpositioned8) {
                laser_line_extraction::LineSegmentList leftWalls;
                laser_line_extraction::LineSegment leftWall;
                laser_line_extraction::LineSegmentList rightWalls;
                laser_line_extraction::LineSegment rightWall;
                laser_line_extraction::LineSegment frontWall = navi->getFrontLine(linesList);

                for(int i = 0;i < numLines;i++) {
                    // Ricerca muro destro
                    if(lines[i].angle < 0) {
                        rightWalls.line_segments.push_back(lines[i]);
                    }//if else
                }//for
                // Sceglie a destra il muro piu' vicino
                if(rightWalls.line_segments.size() > 0) {
                    int minDistanceIndex = 0;
                    double minDistance = rightWalls.line_segments[minDistanceIndex].radius;
                    for(int i = 1;i < rightWalls.line_segments.size();i++) {
                        if(rightWalls.line_segments[i].radius < minDistance) {
                            minDistance = rightWalls.line_segments[i].radius;
                            minDistanceIndex = i;
                        }//if
                    }//for
                    rightWall = rightWalls.line_segments[minDistanceIndex];
                }//if

                for(int i = 0;i < numLines;i++) {
                    // Ricerca muro sinistro
                    if(lines[i].angle > 0) {
                        leftWalls.line_segments.push_back(lines[i]);
                    }//if else
                }//for
                // Sceglie a sinistra il muro piu' vicino
                if(leftWalls.line_segments.size() > 0) {
                    int minDistanceIndex = 0;
                    double minDistance = leftWalls.line_segments[minDistanceIndex].radius;
                    for(int i = 1;i < leftWalls.line_segments.size();i++) {
                        if(leftWalls.line_segments[i].radius < minDistance) {
                            minDistance = leftWalls.line_segments[i].radius;
                            minDistanceIndex = i;
                        }//if
                    }//for
                    leftWall = leftWalls.line_segments[minDistanceIndex];
                }//if

                if(debug) {
                    std::cout << "Muro sinistro: " << leftWall.angle << " " << leftWall.radius << std::endl;
                    std::cout << "Muro frontale: " << frontWall.angle << " " << frontWall.radius << std::endl;
                    std::cout << "Muro destro: " << rightWall.angle << " " << rightWall.radius << std::endl;
                }//if

                if(!rightWallFound8) {
                    if((numLines < 2 && rightWall.radius == 0) || (rightWall.radius != 0 && leftWall.radius > 1)) {
                        // Devo trovare il muro a destra
                        vel.linear.x = 0;
                        vel.angular.z = -0.1;
                        navi->setRobotVelocity(vel, 1, true);
                    } else {
                        // Muro a destra trovato
                        lock();
                        ROS_INFO_STREAM("Muro destro trovato");
                        vel.linear.x = 0;
                        vel.angular.z = 0;
                        navi->setRobotVelocity(vel, 1, true);
                        rightWallFound8 = true;
                        unlock();
                    }//if else
                } else if(!rotated8) {
                    navi->mnav_turnAngleRadians(fabs(rightWall.angle - (utils->getParam("iah8_right_angle"))), -1);
                    lock();
                    vel.linear.x = 0;
                    vel.angular.z = 0;
                    navi->setRobotVelocity(vel, 1, true);
                    rotated8 = true;
                    ROS_INFO_STREAM("Ruotato");
                    unlock();
                } else if(!chosen8) {
                    if(rightWall.radius > utils->getParam("iah8_right_distance") + utils->getParam("EPSILON")) {
                        navi->mnav_goStraight(1, 1);
                    } else if(rightWall.radius < utils->getParam("iah8_right_distance") - utils->getParam("EPSILON")) {
                        navi->mnav_goStraight(1, -1);
                    } else {
                        lock();
                        vel.linear.x = 0;
                        vel.angular.z = 0;
                        navi->setRobotVelocity(vel, 1, true);
                        chosen8 = true;
                        ROS_INFO_STREAM("Avvicinato");
                        unlock();
                    }//if else
                } else if(!leftWallFound8) {
                    if(numLines < 2 && leftWall.radius == 0) {
                        // Devo trovare il muro a sinistra
                        vel.linear.x = 0;
                        vel.angular.z = 0.1;
                        navi->setRobotVelocity(vel, 1, true);
                    } else {
                        // Muro a sinistra trovato
                        ROS_INFO_STREAM("Muro sinistro trovato");
                        lock();
                        vel.linear.x = 0;
                        vel.angular.z = 0;
                        navi->setRobotVelocity(vel, 1, true);
                        leftWallFound8 = true;
                        unlock();
                    }//if else
                } else if(!turned8) {
                    navi->mnav_turnAngleRadians(leftWall.angle, 1, 1);
                    lock();
                    vel.linear.x = 0;
                    vel.angular.z = 0;
                    navi->setRobotVelocity(vel, 1, true);
                    turned8 = true;
                    ROS_INFO_STREAM("Ruotato di nuovo");
                    unlock();
                } else {
                    lock();
                    vel.linear.x = 0;
                    vel.angular.z = 0;
                    navi->setRobotVelocity(vel, 1, true);
                    wellpositioned8 = true;
                    ROS_INFO_STREAM("Parallelo al muro! Ora si avvicina");
                    unlock();
                }//if else
            }//if
        } else {
            // Negli altri casi non e' necessario l'uso dell'estrattore di linee
        }//if else
    }//if
    // Nulla da fare se non vengono rilevate linee
    unlock();
}//laserLineCallback


/**
 * Funzione callback per la visualizzazione delle distanze calcolate con le letture del laser (solo rilevazione, non da comandi ai motori)
 * Utilizzata solo in modalita' sensors_only_mode al posto di executionCallback
 * @param  scanPtr_  puntatore alla variabile contenente la scansione del laser
 */
void laserSenseCallback(const sensor_msgs::LaserScanConstPtr& scanPtr_) {
    if(linesProcessing)
        return;

    lockLaser();

    sensor_msgs::LaserScan scan = *scanPtr_;

    // DESTRA
    double scanDXa = navi->getLaserRightDistanceA(scan);
    double scanDXb = navi->getLaserRightDistanceB(scan);
    double scanDXc = navi->getLaserRightDistanceC(scan);
    double scanDXd = navi->getLaserRightDistanceD(scan);

    // SINISTRA
    double scanSXa = navi->getLaserLeftDistanceA(scan);
    double scanSXb = navi->getLaserLeftDistanceB(scan);
    double scanSXc = navi->getLaserLeftDistanceC(scan);
    double scanSXd = navi->getLaserLeftDistanceD(scan);

    // FRONTALE
    double scanFront = navi->getLaserFrontDistance(scan);

    std::cout << "Scan DXa = " << scanDXa << std::endl;
    std::cout << "Scan DXb = " << scanDXb << std::endl;
    std::cout << "Scan DXc = " << scanDXc << std::endl;
    std::cout << "Scan DXd = " << scanDXd << std::endl;
    std::cout << "Scan Front = " << scanFront << std::endl;
    std::cout << "Scan SXd = " << scanSXd << std::endl;
    std::cout << "Scan SXc = " << scanSXc << std::endl;
    std::cout << "Scan SXb = " << scanSXb << std::endl;
    std::cout << "Scan SXa = " << scanSXa << std::endl;

    std::cout << std::endl << std::endl;

    unlockLaser();
}//laserSenseCallback


/**
 * Funzione callback per la rilevazione delle linee dalla lettura del laser (solo rilevazione, non da comandi ai motori)
 * Utilizzata solo in modalita' sensors_only_mode al posto di laserLineCallback
 * @param  linesListPtr_  puntatore alla variabile contenente la lista delle linee rilevate
 */
void laserLineSenseCallback(const laser_line_extraction::LineSegmentListConstPtr& linesListPtr_) {
    if(laserProcessing)
        return;

    lockLines();

    laser_line_extraction::LineSegmentList linesList = *linesListPtr_;
    std::vector<laser_line_extraction::LineSegment> lines = linesList.line_segments;
    int numLines = linesList.line_segments.size();

    std::cout << "Linee: " << numLines << std::endl;
    for(int i = 0;i < numLines;i++) {
        std::cout << i << ": " << linesList.line_segments[i].angle << " " << linesList.line_segments[i].radius << std::endl;
    }//for

    std::cout << std::endl << std::endl;

    unlockLines();
}//laserLineSenseCallback


/**
 * Punto di inizio dell'esecuzione
 * @param  argc  numero di parametri
 * @param  argv  stringhe rappresentanti i parametri
 * @return  il codice di stato alla fine dell'esecuzione
 */
int main(int argc, char **argv) {

    // Inizializzazione ROS
    ros::init(argc, argv, "hw3");

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

    if(nh.hasParam("is360laser")) {
		nh.getParam("is360laser", is360laser);
	}//if

    if(nh.hasParam("cmd_vel_topic")) {
		nh.getParam("cmd_vel_topic", cmd_vel_topic);
	}//if

    if(nh.hasParam("sensors_only_mode")) {
        nh.getParam("sensors_only_mode", sensors_only_mode);
    }//if

    if(nh.hasParam("force_rotation")) {
        nh.getParam("force_rotation", forceRotation);
    }//if

    // Creazione istanze per uso della libreria di utilita' e di navigazione
    utils = new ros_hw_utils::ROSUtils(nh, simulated);
    navi = new ros_hw_utils::ROSNavigation(nh, simulated, is360laser);

    // Lettura parametri di navigazione
    utils->loadConfigParams();
    navi->setUtils(utils);

    // Comunicazione del proprio stato "Pronto" al server
    ros::ServiceClient clientNavigationReady = nh.serviceClient<g06_fsm::NavigationReady>("/NavigationReady");
    g06_fsm::NavigationReady serviceNR;
    serviceNR.request.iamready = 1;
    if(clientNavigationReady.call(serviceNR)) {
        if(serviceNR.response.received == 1) {
            ROS_INFO_STREAM("Tutto ok! NavigationReady");
            iamhere = serviceNR.response.startiamhere;
        } else {
            iamhere = 0;
        }//if else
    } else {
        ROS_ERROR_STREAM("Errore! NavigationReady");
    }//if else

    // Vengono usati 3 thread
    ros::MultiThreadedSpinner spinner(3);

    // Creazione Subscribers
    ros::Subscriber lineSub;
    ros::Subscriber scanSub;

    if(!sensors_only_mode) {
        // Creazione subscriber che riceve le linee rilevate con laser_line_extraction
        lineSub = nh.subscribe<laser_line_extraction::LineSegmentList>("/line_segments", 1, laserLineCallback);
        // Creazione subscriber che regola l'esecuzione del nodo
        scanSub = nh.subscribe<sensor_msgs::LaserScan>("/marrtino/scan", 1, executionCallback);
    } else {
        lineSub = nh.subscribe<laser_line_extraction::LineSegmentList>("/line_segments", 1, laserLineSenseCallback);
        scanSub = nh.subscribe<sensor_msgs::LaserScan>("/marrtino/scan", 1, laserSenseCallback);
    }//if else

    spinner.spin();

    return 0;
}//main