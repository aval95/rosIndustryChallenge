#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <sensor_msgs/LaserScan.h>
#include <unistd.h>

// Indica se l'esecuzione avviene in simulazione oppure no
bool simulated = true;

// Publishers dei valori elaborati del laser
ros::Publisher m_laser_pub_front;
ros::Publisher m_laser_pub_rear;
ros::Publisher m_laser_pub_left;
ros::Publisher m_laser_pub_right;


/**
 * Calcola (num1) mod (num2) considerando anche il caso di num1 negativo
 * ed ottenendo un risultato diverso da quello ottenuto con l'operatore %
 * Es. -1 % 360 = -1    neg_mod(-1,360) = 359
 * @param  num1  il numero di cui calcolare il modulo
 * @param  num2  la base del modulo da utilizzare
 * @return  num1 mod num2 (come descritto precedentemente)
 */
int neg_mod(int num1, int num2) {
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


/**
 * Metodo necessario per togliere i valori fuori dal range del sensore
 * @param  inputScan  la lettura del laser
 * @return  la lettura del laser dopo il preprocessing
 */
sensor_msgs::LaserScan preprocessLaserData(sensor_msgs::LaserScan inputScan) {

    sensor_msgs::LaserScan inputScanWOinf;
    inputScanWOinf = inputScan;

    // Toglie i paletti impostando le letture nelle loro posizioni a infinito, cosi' poi verranno sostituiti dall'interpolazione
    if(simulated) {
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
 * Funzione che pubblica nel topic /g06/newlaser_front i punti frontali ottenuti con il laser dopo il preprocessing
 * @param  inputScan  le letture grezze del laser
 * @param  nh  istanza di NodeHandle
 * @return  i punti frontali ottenuti con il laser dopo il preprocessing
 */
sensor_msgs::LaserScan publishFrontLaserData(sensor_msgs::LaserScan inputScan, ros::NodeHandle nh) {
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

    m_laser_pub_front = nh.advertise<sensor_msgs::LaserScan>("/g06/newlaser_front", 1);
    usleep(10);
    ros::Time now = ros::Time::now();
    while(ros::ok()){
        m_laser_pub_front.publish(outputScan);
        usleep(10);
	    break;
    }//while
    return outputScan;
}//publishFrontLaserData


/**
 * Funzione che pubblica nel topic /g06/newlaser_rear i punti posteriori ottenuti con il laser dopo il preprocessing
 * @param  inputScan  le letture grezze del laser
 * @param  nh  istanza di NodeHandle
 * @return  i punti posteriori ottenuti con il laser dopo il preprocessing
 */
sensor_msgs::LaserScan publishRearLaserData(sensor_msgs::LaserScan inputScan, ros::NodeHandle nh) {
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

    m_laser_pub_rear = nh.advertise<sensor_msgs::LaserScan>("/g06/newlaser_rear", 1);
    usleep(10);
    ros::Time now = ros::Time::now();
    while(ros::ok()){
        m_laser_pub_rear.publish(outputScan);
        usleep(10);
	    break;
    }//while
    return outputScan;
}//publishRearLaserData


/**
 * Funzione che pubblica nel topic /g06/newlaser_right i punti destri ottenuti con il laser dopo il preprocessing
 * @param  inputScan  le letture grezze del laser
 * @param  nh  istanza di NodeHandle
 * @return  i punti destri ottenuti con il laser dopo il preprocessing
 */
sensor_msgs::LaserScan publishRightLaserData(sensor_msgs::LaserScan inputScan, ros::NodeHandle nh) {
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

    m_laser_pub_right = nh.advertise<sensor_msgs::LaserScan>("/g06/newlaser_right", 1);
    usleep(10);
    ros::Time now = ros::Time::now();
    while(ros::ok()){
        m_laser_pub_right.publish(outputScan);
        usleep(10);
	    break;
    }//while
    return outputScan;
}//publishRightLaserData


/**
 * Funzione che pubblica nel topic /g06/newlaser_left i punti sinistri ottenuti con il laser dopo il preprocessing
 * @param  inputScan  le letture grezze del laser
 * @param  nh  istanza di NodeHandle
 * @return  i punti sinistri ottenuti con il laser dopo il preprocessing
 */
sensor_msgs::LaserScan publishLeftLaserData(sensor_msgs::LaserScan inputScan, ros::NodeHandle nh) {
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

    m_laser_pub_left = nh.advertise<sensor_msgs::LaserScan>("/g06/newlaser_left", 1);
    usleep(10);
    ros::Time now = ros::Time::now();
    while(ros::ok()){
        m_laser_pub_left.publish(outputScan);
        usleep(10);
	    break;
    }//while
    return outputScan;
}//publishLeftLaserData


/**
 * Funzione callback che riceve le letture del laser e le invia alle funzioni di pubblicazione
 * @param  scan  le letture grezze del laser
 */
void executionCallback(const sensor_msgs::LaserScanConstPtr& scan) {
    ros::NodeHandle nh;
    publishFrontLaserData(*scan, nh);
    publishRearLaserData(*scan, nh);
    publishRightLaserData(*scan, nh);
    publishLeftLaserData(*scan, nh);
}//executionCallback


/**
 * Punto di inizio dell'esecuzione
 * @param  argc  numero di parametri
 * @param  argv  stringhe rappresentanti i parametri
 * @return  il codice di stato alla fine dell'esecuzione
 */
int main(int argc, char **argv) {

    // Inizializzazione ROS
    ros::init(argc, argv, "hw3_laserProcessing");

    ros::NodeHandle nh;

    // Lettura parametri
    if(nh.hasParam("sim")) {
		nh.getParam("sim", simulated);
	}//if
	std::cout << "STARTS ";
	if(!simulated)
		std::cout << "NOT ";
	std::cout << "SIMULATED" << std::endl;

    // Creazione e inizializzazione del Subscriber per le letture del laser
    ros::Subscriber scanSub = nh.subscribe<sensor_msgs::LaserScan>("/marrtino/scan", 1, executionCallback);

    ros::spin();
    
}//main