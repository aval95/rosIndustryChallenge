#include <ros/ros.h>
#include <unistd.h>
#include <cmath>

// Servizi
#include <g06_fsm/UserReady.h>
#include <g06_fsm/UserToFSM.h>

// Indica se l'esecuzione avviene in simulazione oppure no
bool simulated = true;


/**
 * Punto di inizio dell'esecuzione
 * @param  argc  numero di parametri
 * @param  argv  stringhe rappresentanti i parametri
 * @return  il codice di stato alla fine dell'esecuzione
 */
int main(int argc, char **argv) {

    // Inizializzazione ROS
    ros::init(argc, argv, "hw4_user");

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

    ros::ServiceClient clientUserReady = nh.serviceClient<g06_fsm::UserReady>("/UserReady");
    ros::ServiceClient clientUserToFSM = nh.serviceClient<g06_fsm::UserToFSM>("/UserToFSM");

    while(true) {
        int selection;
        // Comunicazione del proprio stato "Pronto" al server
        g06_fsm::UserReady serviceUR;
        serviceUR.request.iamready = 1;
        if(clientUserReady.call(serviceUR)) {
            if(serviceUR.response.received == 1) {
                ROS_INFO_STREAM("Tutto ok! UserReady");
            } else {
                ROS_ERROR_STREAM("Errore! UserReady");
                return 1;
            }//if else
        } else {
            ROS_ERROR_STREAM("Errore! UserReady");
            return 1;
        }//if else

        std::cout << "Scrivere 1 quando pronto: ";
        std::cin >> selection;

        if(selection == 1) {
            // Comunicazione dell'avvenuto scaricamento al server
            g06_fsm::UserToFSM serviceU2FSM;
            serviceU2FSM.request.done = 1;
            if(clientUserToFSM.call(serviceU2FSM)) {
                if(serviceU2FSM.response.ack == 1) {
                    ROS_INFO_STREAM("Tutto ok! UserToFSM");
                } else {
                    ROS_ERROR_STREAM("Errore! UserToFSM");
                }//if else
            } else {
                ROS_ERROR_STREAM("Errore! UserToFSM");
            }//if else
        } else {
            break;
        }//if else
        ros::spinOnce();
    }//while

    return 0;
}//main