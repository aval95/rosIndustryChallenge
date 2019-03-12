#include <g06_navigation/ROSNavigation.h>



/**
 * Costruttore, inizializza le variabili d'istanza
 * @param  nodehandle  l'oggetto NodeHandle relativo al nodo corrente
 */
ros_hw_utils::ROSNavigation::ROSNavigation(ros::NodeHandle nodehandle, bool simulated, bool is360laser) : m_nh(nodehandle), m_simulated(simulated), m_is_360degrees_laser(is360laser) {
    m_nh.getParam("odom_topic", m_odom_topic);
    m_nh.getParam("cmd_vel_topic", m_cmd_vel_topic);
    m_vel_pub = m_nh.advertise <geometry_msgs::Twist>(m_cmd_vel_topic, 1);
    m_velocity.linear.x = m_linear;
    m_velocity.linear.y = 0;
    m_velocity.linear.z = 0;
    m_velocity.angular.x = 0;
    m_velocity.angular.y = 0;
    m_velocity.angular.z = m_angular;
}//ROSNavigation


/**
 * Importa un'istanza delle utilita'
 * @param  utils  l'istanza delle utilita'
 */
void ros_hw_utils::ROSNavigation::setUtils(ros_hw_utils::ROSUtils *utils) {
    m_utils = utils;
}//setUtils


/**
 * Metodo per ottenere la posizione attuale stimata del robot mobile
 * @return  la posizione attuale stimata del robot mobile con la sua covarianza
 */
geometry_msgs::PoseWithCovariance ros_hw_utils::ROSNavigation::getCurrentRobotPosition(bool useOdom) {
    if(useOdom) {
        nav_msgs::OdometryConstPtr odomMsgPtr = ros::topic::waitForMessage<nav_msgs::Odometry>(m_odom_topic, m_nh);
        return odomMsgPtr->pose;
    } else {
        geometry_msgs::PoseWithCovarianceStampedConstPtr amclMsgPtr = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("marrtino/amcl_pose", m_nh);
        return amclMsgPtr->pose;
    }//if else
}//getCurrentRobotPosition


/**
 * Metodo per indicare al planner la posizione da raggiungere
 * @param  pose  la posizione del goal
 * @param  execute  indica se eseguire l'azione (default = false)
 * @return  la posizione del goal da raggiungere
 */
geometry_msgs::Pose ros_hw_utils::ROSNavigation::setRobotGoalPosition(geometry_msgs::Pose pose, bool execute) {
    ros::Publisher pub = m_nh.advertise<geometry_msgs::PoseStamped>("/marrtino/move_base_simple/goal", 1);
    sleep(1);
    geometry_msgs::PoseStamped posestamped;
    posestamped.pose = pose;
    posestamped.header.frame_id = "marrtino_map";
    posestamped.header.stamp = ros::Time::now();
    if(execute) {
        while(ros::ok()){
            ros::Rate r(100);
            while(pub.getNumSubscribers() == 0) {
                r.sleep();
            }//while
            pub.publish(posestamped);
            sleep(1);
            break;
        }//while
    }//if
}//setRobotGoalPosition


/**
 * Metodo per ricavare la velocita' attuale del robot dai dati dell'odometria
 * @return  la velocita' attuale con la covarianza
 */
geometry_msgs::TwistWithCovariance ros_hw_utils::ROSNavigation::getCurrentRobotVelocity() {
    nav_msgs::OdometryConstPtr odomMsgPtr = ros::topic::waitForMessage<nav_msgs::Odometry>(m_odom_topic, m_nh);
    return odomMsgPtr->twist;
}//getCurrentRobotVelocity


/**
 * Invia al robot una velocita' da applicare
 * @param  twist  la velocita'
 * @param  time  indica per quanto tempo pubblicare il messaggio contnente la velocita'
 * @param  execute  indica se eseguire l'azione oppure no (default = false)
 * @return  la velocita' inviata
 */
geometry_msgs::Twist ros_hw_utils::ROSNavigation::setRobotVelocity(geometry_msgs::Twist twist, const double time, bool execute) {
    ros::Publisher pub = m_nh.advertise<geometry_msgs::Twist>(m_cmd_vel_topic, 1);
    usleep(10);
    ros::Time now = ros::Time::now();
	while((ros::Time::now () - now).sec < time){
		ros::Rate r(100);
		while(pub.getNumSubscribers() == 0) {
			r.sleep();
		}//while
		pub.publish(twist);
        ros::spinOnce();
	}//while
}//setRobotVelocity


/**
 * Metodo per ottenere una lettura singola dei dati del laser
 * @return  la lettura del laser
 */
sensor_msgs::LaserScan ros_hw_utils::ROSNavigation::getLaserScan() {
    sensor_msgs::LaserScanConstPtr laserScanPtr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/marrtino/scan", m_nh);
    return *laserScanPtr;
}//getLaserScan


/**
 * Metodo necessario per togliere i valori fuori dal range del sensore
 * @param  inputScan  la lettura del laser
 * @return  la lettura del laser dopo il preprocessing
 */
sensor_msgs::LaserScan ros_hw_utils::ROSNavigation::preprocessLaserData(sensor_msgs::LaserScan inputScan) {
    sensor_msgs::LaserScan inputScanWOinf;
    inputScanWOinf = inputScan;

    // Toglie i paletti impostando le letture nelle loro posizioni a infinito, cosi' poi verranno sostituiti dall'interpolazione
    if(m_simulated) {
        inputScanWOinf.ranges[36] = inputScanWOinf.ranges[37] = inputScanWOinf.ranges[38] = std::numeric_limits<float>::infinity();
        inputScanWOinf.ranges[99] = inputScanWOinf.ranges[100] = inputScanWOinf.ranges[101] = inputScanWOinf.ranges[102] = std::numeric_limits<float>::infinity();
        inputScanWOinf.ranges[257] = inputScanWOinf.ranges[258] = inputScanWOinf.ranges[259] = inputScanWOinf.ranges[260] = std::numeric_limits<float>::infinity();
        inputScanWOinf.ranges[321] = inputScanWOinf.ranges[322] = inputScanWOinf.ranges[323] = std::numeric_limits<float>::infinity();
    } else {
        for(int i = 30;i < 35;i++) {
            if(inputScanWOinf.ranges[i] < 0.31) {
                inputScanWOinf.ranges[i] = std::numeric_limits<float>::infinity();
            }//if
        }//for
        for(int i = 106;i < 112;i++) {
            if(inputScanWOinf.ranges[i] < 0.18) {
                inputScanWOinf.ranges[i] = std::numeric_limits<float>::infinity();
            }//if
        }//for
        for(int i = 248;i < 254;i++) {
            if(inputScanWOinf.ranges[i] < 0.18) {
                inputScanWOinf.ranges[i] = std::numeric_limits<float>::infinity();
            }//if
        }//for
        for(int i = 320;i < 325;i++) {
            if(inputScanWOinf.ranges[i] < 0.31) {
                inputScanWOinf.ranges[i] = std::numeric_limits<float>::infinity();
            }//if
        }//for
    }//if else
    
    // Toglie gli infiniti sostituendo l'interpolazione con i punti adiacenti
    int DUMMY_VALUE = 10;
    int numValues = inputScanWOinf.ranges.size();
    int i = 0;
    while(i < numValues) {
        if(inputScanWOinf.ranges[i] == std::numeric_limits<float>::infinity()) {
            int indexBack = i;
            int indexBackNoMod = i;
            int numIter = 0;
            // indexBack sara' l'indice della prima posizione con valore inf
            while(inputScanWOinf.ranges[indexBack] == std::numeric_limits<float>::infinity() && numIter <= numValues) {
                indexBack = neg_mod((indexBack - 1), numValues);
                indexBackNoMod = indexBackNoMod - 1;
                numIter++;
            }//while
            int firstIndex = neg_mod((indexBack + 1), numValues);
            indexBackNoMod++;
            int indexForward = firstIndex;
            int indexForwardNoMod = indexBackNoMod;
            //std::cout << firstIndex << " " << indexBackNoMod << " - ";
            numIter = 0;
            // indexForward sara' l'indice della prima posizione con valore non inf dopo la lista di inf
            do {
                indexForward = neg_mod((indexForward + 1), numValues);
                indexForwardNoMod = indexForwardNoMod + 1;
                numIter++;
            } while(inputScanWOinf.ranges[indexForward] == std::numeric_limits<float>::infinity() && numIter <= numValues);
            int lastIndex = indexForward;
            //std::cout << lastIndex << " " << indexForwardNoMod << " - ";
            int infCount = abs(indexForwardNoMod - indexBackNoMod);
            double diff = inputScanWOinf.ranges[lastIndex] - inputScanWOinf.ranges[neg_mod((firstIndex - 1), numValues)];
            //std::cout << infCount << " " << diff << std::endl;
            for(int j = 0;j < infCount;j++) {
                inputScanWOinf.ranges[neg_mod((firstIndex + j), numValues)] = inputScanWOinf.ranges[neg_mod((firstIndex - 1), numValues)] + (j+1) * (diff / (double) (infCount + 1));
            }//for
            i = lastIndex;
        } else {
            i++;
        }//if else
    }//while

    return inputScanWOinf;
}//preprocessLaserData


/**
 * Restituisce i punti frontali ottenuti con il laser dopo il preprocessing
 * @param  inputScan  le letture grezze del laser
 * @return  i punti frontali ottenuti con il laser dopo il preprocessing
 */
sensor_msgs::LaserScan ros_hw_utils::ROSNavigation::getFrontLaserData(sensor_msgs::LaserScan inputScan) {
    // Riceve 360 valori della vista in 360 gradi, devo togliere la parte posteriore e tenere solo 150 gradi davanti
    sensor_msgs::LaserScan outputScan;
    outputScan = inputScan;
    outputScan.ranges.resize(150);
    outputScan.intensities.resize(150);
    outputScan.angle_min = -1.308996938;
    outputScan.angle_max = 1.308996938;

    inputScan = preprocessLaserData(inputScan);  

    int j = 105;
    for(int i = 0;i < outputScan.ranges.size();i++,j++) {
        outputScan.ranges[i] = inputScan.ranges[j];
        outputScan.intensities[i] = inputScan.intensities[j];
    }//for

    return outputScan;
}//getFrontLaserData


/**
 * Restituisce i punti posteriori ottenuti con il laser dopo il preprocessing
 * @param  inputScan  le letture grezze del laser
 * @return  i punti posteriori ottenuti con il laser dopo il preprocessing
 */
sensor_msgs::LaserScan ros_hw_utils::ROSNavigation::getRearLaserData(sensor_msgs::LaserScan inputScan) {
    // Riceve 360 valori della vista in 360 gradi, devo togliere la parte frontale e tenere solo 90 gradi dietro
    sensor_msgs::LaserScan outputScan;
    outputScan = inputScan;
    outputScan.ranges.resize(90);
    outputScan.intensities.resize(90);
    outputScan.angle_min = 2.35619449;
    outputScan.angle_max = -2.35619449;

    inputScan = preprocessLaserData(inputScan);  

    int i;
    int j;
    for(i = 0, j = 315;i < (outputScan.ranges.size()) / 2;i++,j++) {
        outputScan.ranges[i] = inputScan.ranges[j];
        outputScan.intensities[i] = inputScan.intensities[j];
    }//for
    for(j = 0;i < outputScan.ranges.size();i++,j++) {
        outputScan.ranges[i] = inputScan.ranges[j];
        outputScan.intensities[i] = inputScan.intensities[j];
    }//for

    return outputScan;
}//getRearLaserData


/**
 * Restituisce i punti destri ottenuti con il laser dopo il preprocessing
 * @param  inputScan  le letture grezze del laser
 * @return  i punti destri ottenuti con il laser dopo il preprocessing
 */
sensor_msgs::LaserScan ros_hw_utils::ROSNavigation::getRightLaserData(sensor_msgs::LaserScan inputScan) {
    // Riceve 360 valori della vista in 360 gradi, devo tenere solo la parte destra
    sensor_msgs::LaserScan outputScan;
    outputScan = inputScan;
    outputScan.ranges.resize(20);
    outputScan.intensities.resize(20);
    outputScan.angle_min = -1.919862177;
    outputScan.angle_max = -1.570796327;

    inputScan = preprocessLaserData(inputScan);

    int j = 69;
    for(int i = 0;i < outputScan.ranges.size();i++,j++) {
        outputScan.ranges[i] = inputScan.ranges[j];
        outputScan.intensities[i] = inputScan.intensities[j];
    }//for

    return outputScan;
}//getRightLaserData


/**
 * Restituisce i punti sinistri ottenuti con il laser dopo il preprocessing
 * @param  inputScan  le letture grezze del laser
 * @return  i punti sinistri ottenuti con il laser dopo il preprocessing
 */
sensor_msgs::LaserScan ros_hw_utils::ROSNavigation::getLeftLaserData(sensor_msgs::LaserScan inputScan) {
    // Riceve 360 valori della vista in 360 gradi, devo tenere solo la parte sinistra
    sensor_msgs::LaserScan outputScan;
    outputScan = inputScan;
    outputScan.ranges.resize(20);
    outputScan.intensities.resize(20);
    outputScan.angle_min = 1.570796327;
    outputScan.angle_max = 1.919862177;

    inputScan = preprocessLaserData(inputScan);

    int j = 269;
    for(int i = 0;i < outputScan.ranges.size();i++,j++) {
        outputScan.ranges[i] = inputScan.ranges[j];
        outputScan.intensities[i] = inputScan.intensities[j];
    }//for

    return outputScan;
}//getLeftLaserData


/**
 * Restituisce il valore di distanza letto dal laser nella posizione indicata dal parametro
 * @param  scanID  l'indice del valore da restituire
 * @return  il valore di distanza, se l'indice e' valido
 */
double ros_hw_utils::ROSNavigation::getLaserScan(int scanID) {
    sensor_msgs::LaserScan scan = getLaserScan();
    if(scanID >= 0 && scanID < scan.ranges.size()) {
        return scan.ranges[scanID];
    }//if
}//getLaserScan


/**
 * Restituisce la distanza letta dal laser dai raggi ricevuti a destra
 * @param  numberOfValuesForMean  numero di raggi a destra da usare per mediare il valore (default = 40)
 * @return  la distanza a destra
 */
double ros_hw_utils::ROSNavigation::getLaserRightDistance(int numberOfValuesForMean) {
    return getLaserRightDistance(getLaserScan(), numberOfValuesForMean);
}//getLaserRightDistance


/**
 * Restituisce la distanza letta dal laser dai raggi ricevuti a sinistra
 * @param  numberOfValuesForMean  numero di raggi a sinistra da usare per mediare il valore (default = 40)
 * @return  la distanza a sinistra
 */
double ros_hw_utils::ROSNavigation::getLaserLeftDistance(int numberOfValuesForMean) {
    return getLaserLeftDistance(getLaserScan(), numberOfValuesForMean);
}//getLaserLeftDistance


/**
 * Restituisce la distanza letta dal laser dai raggi ricevuti frontalmente
 * @param  numberOfValuesForMean  numero di raggi frontali da usare per mediare il valore (default = 80)
 * @return  la distanza frontale
 */
double ros_hw_utils::ROSNavigation::getLaserFrontDistance(int numberOfValuesForMean) {
    return getLaserFrontDistance(getLaserScan(), numberOfValuesForMean);
}//getLaserFrontDistance


/**
 * Restituisce la distanza letta dal laser dai raggi ricevuti a destra (usando una lettura data invece di istantanea)
 * @param  inputScan  la lettura del laser da cui ricavare i dati
 * @param  numberOfValuesForMean  numero di raggi a destra da usare per mediare il valore (default = 40)
 * @return  la distanza a destra
 */
double ros_hw_utils::ROSNavigation::getLaserRightDistance(sensor_msgs::LaserScan inputScan, int numberOfValuesForMean) {
    if(m_is_360degrees_laser) {
        inputScan = getFrontLaserData(inputScan);
    }//if

    int numScan = inputScan.ranges.size();

    if(numberOfValuesForMean >= numScan) {
        numberOfValuesForMean = 40;
    }//if

    double sumScan = 0;
    for(int i = 0;i < numberOfValuesForMean;i++) {
        sumScan += inputScan.ranges[i];
    }//for
    return sumScan / numberOfValuesForMean;
}//getLaserRightDistance


/**
 * Restituisce la distanza letta dal laser dai raggi ricevuti a sinistra (usando una lettura data invece di istantanea)
 * @param  inputScan  la lettura del laser da cui ricavare i dati
 * @param  numberOfValuesForMean  numero di raggi a sinistra da usare per mediare il valore (default = 40)
 * @return  la distanza a sinistra
 */
double ros_hw_utils::ROSNavigation::getLaserLeftDistance(sensor_msgs::LaserScan inputScan, int numberOfValuesForMean) {
    if(m_is_360degrees_laser) {
        inputScan = getFrontLaserData(inputScan);
    }//if

    int numScan = inputScan.ranges.size();

    if(numberOfValuesForMean >= numScan) {
        numberOfValuesForMean = 40;
    }//if

    double sumScan = 0;
    for(int i = 0;i < numberOfValuesForMean;i++) {
        sumScan += inputScan.ranges[numScan - 1 - i];
    }//for
    return sumScan / numberOfValuesForMean;
}//getLaserLeftDistance


/**
 * Restituisce la distanza letta dal laser dai raggi ricevuti frontalmente (usando una lettura data invece di istantanea)
 * @param  inputScan  la lettura del laser da cui ricavare i dati
 * @param  numberOfValuesForMean  numero di raggi frontali da usare per mediare il valore (default = 80)
 * @return  la distanza frontale
 */
double ros_hw_utils::ROSNavigation::getLaserFrontDistance(sensor_msgs::LaserScan inputScan, int numberOfValuesForMean) {
    if(m_is_360degrees_laser) {
        inputScan = getFrontLaserData(inputScan);
    }//if

    int numScan = inputScan.ranges.size();

    if(numberOfValuesForMean >= numScan) {
        numberOfValuesForMean = m_is_360degrees_laser ? 24 : 80;
    }//if

    double sumScan = 0;
    for(int i = 0;i < numberOfValuesForMean;i++) {
        sumScan += inputScan.ranges[numScan / 2 - (numberOfValuesForMean / 2) + i];
    }//for
    return sumScan / numberOfValuesForMean;
}//getLaserFrontDistance


/**
 * Restituisce la distanza letta dal laser dai raggi ricevuti a destra di tipo A (usando una lettura data invece di istantanea)
 * Vedi documentazione per spiegazione circa la suddivisione della lettura del laser
 * @param  inputScan  la lettura del laser da cui ricavare i dati
 * @return  la distanza a destra di tipo A
 */
double ros_hw_utils::ROSNavigation::getLaserRightDistanceA(sensor_msgs::LaserScan inputScan) {
    int MIN = 0;
    int MAX = 40;
    if(m_is_360degrees_laser) {
        inputScan = getFrontLaserData(inputScan);
        MAX = 12;
    }//if

    double sumScanDXa = 0;
    for(int i = MIN;i < MAX;i++) {
        sumScanDXa += inputScan.ranges[i];
    }//for
    return sumScanDXa / (MAX - MIN);
}//getLaserRightDistanceA


/**
 * Restituisce la distanza letta dal laser dai raggi ricevuti a destra di tipo B (usando una lettura data invece di istantanea)
 * Vedi documentazione per spiegazione circa la suddivisione della lettura del laser
 * @param  inputScan  la lettura del laser da cui ricavare i dati
 * @return  la distanza a destra di tipo B
 */
double ros_hw_utils::ROSNavigation::getLaserRightDistanceB(sensor_msgs::LaserScan inputScan) {
    int MIN = 60;
    int MAX = 140;
    if(m_is_360degrees_laser) {
        inputScan = getFrontLaserData(inputScan);
        MIN = 12;
        MAX = 32;
    }//if

    double sumScanDXb = 0;
    for(int i = MIN;i < MAX;i++) {
        sumScanDXb += inputScan.ranges[i];
    }//for
    return sumScanDXb / (MAX - MIN);
}//getLaserRightDistanceB


/**
 * Restituisce la distanza letta dal laser dai raggi ricevuti a destra di tipo C (usando una lettura data invece di istantanea)
 * Vedi documentazione per spiegazione circa la suddivisione della lettura del laser
 * @param  inputScan  la lettura del laser da cui ricavare i dati
 * @return  la distanza a destra di tipo C
 */
double ros_hw_utils::ROSNavigation::getLaserRightDistanceC(sensor_msgs::LaserScan inputScan) {
    int MIN = 170;
    int MAX = 190;
    if(m_is_360degrees_laser) {
        inputScan = getFrontLaserData(inputScan);
        MIN = 32;
        MAX = 43;
    }//if

    double sumScanDXc = 0;
    for(int i = MIN;i < MAX;i++) {
        sumScanDXc += inputScan.ranges[i];
    }//for
    return sumScanDXc / (MAX - MIN);
}//getLaserRightDistanceC


/**
 * Restituisce la distanza letta dal laser dai raggi ricevuti a destra di tipo D (usando una lettura data invece di istantanea)
 * Vedi documentazione per spiegazione circa la suddivisione della lettura del laser
 * @param  inputScan  la lettura del laser da cui ricavare i dati
 * @return  la distanza a destra di tipo D
 */
double ros_hw_utils::ROSNavigation::getLaserRightDistanceD(sensor_msgs::LaserScan inputScan) {
    int MIN = 220;
    int MAX = 300;
    if(m_is_360degrees_laser) {
        inputScan = getFrontLaserData(inputScan);
        MIN = 43;
        MAX = 63;
    }//if

    double sumScanDXd = 0;
    for(int i = MIN;i < MAX;i++) {
        sumScanDXd += inputScan.ranges[i];
    }//for
    return sumScanDXd / (MAX - MIN);
}//getLaserRightDistanceD


/**
 * Restituisce la distanza letta dal laser dai raggi ricevuti a sinistra di tipo A (usando una lettura data invece di istantanea)
 * Vedi documentazione per spiegazione circa la suddivisione della lettura del laser
 * @param  inputScan  la lettura del laser da cui ricavare i dati
 * @return  la distanza a sinistra di tipo A
 */
double ros_hw_utils::ROSNavigation::getLaserLeftDistanceA(sensor_msgs::LaserScan inputScan) {
    int MIN = 680;
    int MAX = 720;
    if(m_is_360degrees_laser) {
        inputScan = getFrontLaserData(inputScan);
        MIN = 138;
        MAX = 150;
    }//if

    double sumScanSXa = 0;
    for(int i = MIN;i < MAX;i++) {
        sumScanSXa += inputScan.ranges[i];
    }//for
    return sumScanSXa / (MAX - MIN);
}//getLaserLeftDistanceA


/**
 * Restituisce la distanza letta dal laser dai raggi ricevuti a sinistra di tipo B (usando una lettura data invece di istantanea)
 * Vedi documentazione per spiegazione circa la suddivisione della lettura del laser
 * @param  inputScan  la lettura del laser da cui ricavare i dati
 * @return  la distanza a sinistra di tipo B
 */
double ros_hw_utils::ROSNavigation::getLaserLeftDistanceB(sensor_msgs::LaserScan inputScan) {
    int MIN = 580;
    int MAX = 660;
    if(m_is_360degrees_laser) {
        inputScan = getFrontLaserData(inputScan);
        MIN = 118;
        MAX = 138;
    }//if

    double sumScanSXb = 0;
    for(int i = MIN;i < MAX;i++) {
        sumScanSXb += inputScan.ranges[i];
    }//for
    return sumScanSXb / (MAX - MIN);
}//getLaserLeftDistanceB


/**
 * Restituisce la distanza letta dal laser dai raggi ricevuti a sinistra di tipo C (usando una lettura data invece di istantanea)
 * Vedi documentazione per spiegazione circa la suddivisione della lettura del laser
 * @param  inputScan  la lettura del laser da cui ricavare i dati
 * @return  la distanza a sinistra di tipo C
 */
double ros_hw_utils::ROSNavigation::getLaserLeftDistanceC(sensor_msgs::LaserScan inputScan) {
    int MIN = 530;
    int MAX = 550;
    if(m_is_360degrees_laser) {
        inputScan = getFrontLaserData(inputScan);
        MIN = 107;
        MAX = 118;
    }//if

    double sumScanSXc = 0;
    for(int i = MIN;i < MAX;i++) {
        sumScanSXc += inputScan.ranges[i];
    }//for
    return sumScanSXc / (MAX - MIN);
}//getLaserLeftDistanceC


/**
 * Restituisce la distanza letta dal laser dai raggi ricevuti a sinistra di tipo D (usando una lettura data invece di istantanea)
 * Vedi documentazione per spiegazione circa la suddivisione della lettura del laser
 * @param  inputScan  la lettura del laser da cui ricavare i dati
 * @return  la distanza a sinistra di tipo D
 */
double ros_hw_utils::ROSNavigation::getLaserLeftDistanceD(sensor_msgs::LaserScan inputScan) {
    int MIN = 420;
    int MAX = 500;
    if(m_is_360degrees_laser) {
        inputScan = getFrontLaserData(inputScan);
        MIN = 87;
        MAX = 107;
    }//if

    double sumScanSXd = 0;
    for(int i = MIN;i < MAX;i++) {
        sumScanSXd += inputScan.ranges[i];
    }//for
    return sumScanSXd / (MAX - MIN);
}//getLaserLeftDistanceD


/**
 * Indica da quale parte (destra o sinistra) rispetto al robot si trova l'ostacolo piu' vicino
 * @param  inputScan  la lettura del laser da usare per trovare l'ostacolo
 * @return  -1 se l'ostacolo e' a destra, 1 se e' a sinistra
 */
int ros_hw_utils::ROSNavigation::getObstacleRelativePosition(sensor_msgs::LaserScan inputScan) {
    int SECTION_SIZE = 10;
    if(m_is_360degrees_laser) {
        inputScan = getFrontLaserData(inputScan);
        SECTION_SIZE = 5;
    }//if

    // Divide la lettura del laser in sezioni da 10 o 5 raggi e fa la media per ogni sezione
    int numScan = inputScan.ranges.size();
    double laserScan[numScan / SECTION_SIZE];
    for(int i = 0;i < numScan / SECTION_SIZE;i++) {
        double sum = 0;
        for(int j = i * SECTION_SIZE;j < (i+1) * SECTION_SIZE;j++) {
            sum += inputScan.ranges[j];
        }//for
        laserScan[i] = sum / SECTION_SIZE;
    }//for

    // Trova la sezione con distanza media minima
    int indexMin = 0;
    double min = laserScan[0];
    for(int i = 1;i < numScan / SECTION_SIZE;i++) {
        if(laserScan[i] < min) {
            min = laserScan[i];
            indexMin = i;
        }//if
    }//for

    // Dato che le sezioni sono 72, le prime 36 saranno a destra mentre le altre a sinistra
    // Nel caso del reale le sezioni saranno 30, quindi le prime 15 saranno a destra mentre le altre a sinistra
    return (indexMin <= (m_is_360degrees_laser ? 14 : 35) ? -1 : 1);
}//getObstacleRelativePosition



/// Laser Line Extraction

/**
 * Funzione che restituisce la lista delle linee trovate a destra
 * @param  linesList  la lista di tutte le linee trovate
 * @return  la lista delle linee a destra
 */
laser_line_extraction::LineSegmentList ros_hw_utils::ROSNavigation::getRightLines(laser_line_extraction::LineSegmentList linesList) {
    std::vector<laser_line_extraction::LineSegment> lines = linesList.line_segments;
    int numLines = linesList.line_segments.size();

    laser_line_extraction::LineSegmentList rightWalls;
    for(int i = 0;i < numLines;i++) {
        // Ricerca muri destri
            if(lines[i].angle < -M_PI/2 + m_utils->getParam("lateral_line_tolerance")) {
            rightWalls.line_segments.push_back(lines[i]);
        }//if else
    }//for

    return rightWalls;
}//getRightLines


/**
 * Funzione che restituisce la lista delle linee trovate a sinistra
 * @param  linesList  la lista di tutte le linee trovate
 * @return  la lista delle linee a sinistra
 */
laser_line_extraction::LineSegmentList ros_hw_utils::ROSNavigation::getLeftLines(laser_line_extraction::LineSegmentList linesList) {
    std::vector<laser_line_extraction::LineSegment> lines = linesList.line_segments;
    int numLines = linesList.line_segments.size();

    laser_line_extraction::LineSegmentList leftWalls;
    for(int i = 0;i < numLines;i++) {
        // Ricerca muri sinistri
        if(lines[i].angle > M_PI/2 - m_utils->getParam("lateral_line_tolerance")) {
            leftWalls.line_segments.push_back(lines[i]);
        }//if
    }//for

    return leftWalls;
}//getLeftLines


/**
 * Funzione che restituisce la linea frontale rilevata (in base a criteri)
 * @param  linesList  la lista di tutte le linee trovate
 * @return  la linea frontale rilevata
 */
laser_line_extraction::LineSegment ros_hw_utils::ROSNavigation::getFrontLine(laser_line_extraction::LineSegmentList linesList) {
    std::vector<laser_line_extraction::LineSegment> lines = linesList.line_segments;
    int numLines = linesList.line_segments.size();
    
    laser_line_extraction::LineSegment frontWall;
    // Ricerca muro frontale (con angolo minimo)
    int minAngleIndex = 0;
    double minAngle = lines[minAngleIndex].angle;
    for(int i = 1;i < numLines;i++) {
        if(fabs(lines[i].angle) < fabs(minAngle)) {
            minAngle = lines[i].angle;
            minAngleIndex = i;
        }//if
    }//for
    frontWall = lines[minAngleIndex];

    return frontWall;
}//getFrontLine


/**
 * Funzione che restituisce la linea destra rilevata che soddisfa determinati criteri (quella piu' vicina)
 * @param  linesList  la lista di tutte le linee trovate
 * @return  la linea destra rilevata
 */
laser_line_extraction::LineSegment ros_hw_utils::ROSNavigation::getRightLine(laser_line_extraction::LineSegmentList linesList) {
    laser_line_extraction::LineSegmentList rightWalls = getRightLines(linesList);
    laser_line_extraction::LineSegment rightWall;
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
    return rightWall;
}//getRightLine


/**
 * Funzione che restituisce la linea sinistra rilevata che soddisfa determinati criteri (quella piu' vicina)
 * @param  linesList  la lista di tutte le linee trovate
 * @return  la linea sinistra rilevata
 */
laser_line_extraction::LineSegment ros_hw_utils::ROSNavigation::getLeftLine(laser_line_extraction::LineSegmentList linesList) {
    laser_line_extraction::LineSegmentList leftWalls = getLeftLines(linesList);
    laser_line_extraction::LineSegment leftWall;
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
    return leftWall;
}//getLeftLine


/**
 * Calcola (num1) mod (num2) considerando anche il caso di num1 negativo
 * ed ottenendo un risultato diverso da quello ottenuto con l'operatore %
 * Es. -1 % 360 = -1    neg_mod(-1,360) = 359
 * @param  num1  il numero di cui calcolare il modulo
 * @param  num2  la base del modulo da utilizzare
 * @return  num1 mod num2 (come descritto precedentemente)
 */
int ros_hw_utils::ROSNavigation::neg_mod(int num1, int num2) {
    if(num1 >= 0) {
        if(num1 < num2) {
            return num1;
        } else {
            return num1 % num2;
        }//if else
    } else {
        int temp = abs(num1);
        int count = 0;
        while(temp >= num2) {
            count++;
            temp -= num2;
        }//while
        return num2 - ((count * num2) + temp);
    }//if else
}//neg_mod


/// Metodi per la navigazione manuale (mnav = manual navigation)

/**
 * Metodo per far muovere il robot mobile in avanti o indietro
 * @param  time  indica per quanto tempo pubblicare la velocita'
 * @param  sign  1 = avanti, -1 = indietro
 * @param  linear_speed  velocita' lineare lungo l'asse x del robot (default = 0.1)
 */
void ros_hw_utils::ROSNavigation::mnav_goStraight(const double &time, int sign, double linear_speed) {
    ros::Time now = ros::Time::now();
    while((ros::Time::now () - now).sec < time) {
        m_velocity.linear.x = sign * linear_speed;
        m_velocity.angular.z = m_angular;
        m_vel_pub.publish(m_velocity);
    }//while
}//mnav_goStraight


/**
 * Metodo per far muovere il robot mobile in avanti o indietro curvando contemporaneamente
 * @param  time  indica per quanto tempo pubblicare la velocita'
 * @param  sign  1 = avanti, -1 = indietro
 * @param  linear_speed  velocita' lineare lungo l'asse x del robot (default = 0.1)
 * @param  angular_speed  valocita' angolare lungo l'asse z del robot (default = 0.2)
 */
void ros_hw_utils::ROSNavigation::mnav_goStraightWithCurve(const double &time, int sign, double linear_speed, double angular_speed) {
    ros::Time now = ros::Time::now();
    while((ros::Time::now () - now).sec < time) {
        m_velocity.linear.x = sign * linear_speed;
        m_velocity.angular.z = angular_speed;
        m_vel_pub.publish(m_velocity);
    }//while
}//mnav_goStraightWithCurve


/**
 * Metodo per far ruotare il robot mobile a destra o a sinistra
 * @param  time  indica per quanto tempo pubblicare la velocita'
 * @param  sign  1 = sinistra, -1 = destra
 * @param  angular_speed  valocita' angolare lungo l'asse z del robot (default = pigreco/8 = 0.39...)
 */
void ros_hw_utils::ROSNavigation::mnav_turn(const double &rotation_time, int sign, double angular_speed) {
    ros::Time now = ros::Time::now();
    while((ros::Time::now() - now).sec < rotation_time) {
        //ROS_INFO_STREAM ("actual - now: " << ros::Time::now() - now);
        m_velocity.linear.x = 0;
        m_velocity.angular.z = sign * angular_speed;
        m_vel_pub.publish(m_velocity);
    }//while
}//mnav_turn


/**
 * Metodo per far ruotare il robot mobile a destra o a sinistra
 * @param  radians  indica di quanti radianti ruotare il robot
 * @param  sign  1 = sinistra, -1 = destra
 * @param  angular_speed  valocita' angolare lungo l'asse z del robot (default = pigreco/8 = 0.39...)
 * @param  then_stop  indica se fermare il robot dopo la rotazione oppure no
 */
void ros_hw_utils::ROSNavigation::mnav_turnAngleRadians(const double radians, int sign, double angular_speed, bool then_stop) {
    // Calcolo del tempo di rotazione necessario (tempo in secondi)
    double time = radians / angular_speed;
    int64_t time_ns = radians / (angular_speed / 1000000000);
    int32_t needed_sec = (int32_t) (time_ns / 1000000000);
    int32_t needed_nsec = (int32_t) (time_ns % 1000000000);
    //ros::Time::init();
    ros::Time now = ros::Time::now();
    while((ros::Time::now() - now) < ros::Duration(needed_sec, needed_nsec)) {
        //ROS_INFO_STREAM ("actual - now: " << ros::Time::now() - now);
        m_velocity.linear.x = 0;
        m_velocity.angular.z = sign * angular_speed;
        m_vel_pub.publish(m_velocity);
        //ros::spinOnce () ;
    }//while
    if(then_stop) {
        mnav_stop();
    }//if
}//mnav_turnAngleRadians


/**
 * Metodo per far ruotare il robot mobile a destra o a sinistra
 * @param  degrees  indica di quanti gradi ruotare il robot
 * @param  sign  1 = sinistra, -1 = destra
 * @param  angular_speed  valocita' angolare lungo l'asse z del robot (default = pigreco/8 = 0.39...)
 */
void ros_hw_utils::ROSNavigation::mnav_turnAngleDegrees(const double degrees, int sign, double angular_speed, bool then_stop) {
    double radians = degrees * (M_PI / 180.0);  // Conversione da gradi a radianti
    mnav_turnAngleRadians(radians, sign, angular_speed, then_stop);
}//mnav_turnAngleDegrees


/**
 * Metodo per far fermare il robot
 */
void ros_hw_utils::ROSNavigation::mnav_stop() {
    m_velocity.linear.x = 0;
    m_velocity.linear.y = 0;
    m_velocity.linear.z = 0;
    m_velocity.angular.x = 0;
    m_velocity.angular.y = 0;
    m_velocity.angular.z = 0;
    m_vel_pub.publish(m_velocity);
}//mnav_stop