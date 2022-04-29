#include <Arduino.h>
#include <Servo.h>
#include <Stepper.h>
#include "arm.h"
#include <stdio.h>





//#############CONSTANTES ET VARIABLES###########


// Déclaration des ports capteurs.
const int shoulder_cpt = 18;
const int shoulder_cpt_dir = 19;
const int elbow_cpt = 20;
const int elbow_cpt_dir = 21;
// Le port sans nom est le premier port de réception. Le port _dir correspond à l'input donnant le sens de rotation. 


volatile int shoulderEncoderValue = 0; 
volatile int elbowEncoderValue = 0;
// Compteurs pour les encodeurs.
float cpt_shoulder_vit = 0;
float cpt_elbow_vit = 0;
const int encoder_PPR = 2803/4;
//Nombre de cycles d'encodeur par tour de moteur.



// Ports, valeur du capteur, et stockage des vitesses en rad.s-1

const float arm = 25;
const float foreArm = 25;
const float wrist = 10;
// Déclaration des tailles des membres, en cm.
 


const int number_steps = 200;
const int step_angle = 360 / number_steps; // = 1.8 degrés par step, à combiner avec le réducteur de 40.
int steps = 0; //Nombre de steps qu'on demandera au stepper de bouger. 
int stepper_base = 0; //Sauvegarde de la position du stepper. Utilisé lorsque des ordres sont reçus. 



// Variables Serial
char buf[1024]; 
int cur = 0;
// Buffer qui contiendra le message serial reçu. Ce message pourrait être incomplet vu que l'arduino tourne très vite.
// Cur est la position du curseur qui parcourt le message
bool validBuf = false;
// Booléen mettant le programme au courant lorsqu'un buffer complet est reçu.

float vit_alpha = 0;
float vit_beta = 0;
float vit_rotation = 0;
// Vitesses/puissances demandées aux moteurs des angles alpha et beta. Valeurs comprises entre -255 et 255.


//variables de temps pour les capteurs
const long asserve_interval = 20;
// en millisecondes. Temps minimal d'activation entre deux capteurs.
const long serial_interval = 100;
// en millsecondes, Temps minimal d'espacement entre deux réceptions de données.



long loop_duration = 1;
long serial_loop_duration = 1;
// temps en ms s'étant réellement écoulé en us entre les deux activations des capteurs et entre deux réceptions serial. Variable. initialisé à 1 pour éviter des divisions par 0.

long previousMillis = 0;
long currentMillis = 0;
long previousMillisSerial = 0;
// pour compter le temps

// Déclaration des pins MCC
const int shoulderMotorPwm = 5;
const int shoulderMotorDir = 4;
const int elbowMotorPwm = 6;
const int elbowMotorDir = 7;
const int rotationMotorPwm = 3;
const int rotationMotorDir = 2;


// Déclaration du stepper
Stepper* stepperMotor = new Stepper(number_steps, 24, 25, 26, 27);



// Déclaration du Robot
Arm* robot = new Arm(arm, foreArm, wrist);



//###########FONCTIONS#############

// Fonctions pour asservir les moteurs
void asserve_shoulder(){

    int sgn_target = signof(robot->get_alpha_diff());
    // sgn = -1 si l'angle cible est inférieur à l'angle souhaité, et inversement
    int sgn_cpt = signof(cpt_shoulder_vit);

    // On compare les valeurs obsolues
    if (sgn_cpt*(cpt_shoulder_vit) < sgn_target * robot->get_alpha_diff() *  ( 1000 / ( serial_loop_duration ) ) ){
        // Si la vitesse de rotation, en radians en valeur absolue, est inférieure à la valeur absolue de la différence d'angles divisé par la période (ce qui donne une vitesse de rotation)
        // *1000/serial_interval car (robot->alpha - robot->alpha_p) est un angle, on divise par la période d'acquisition des inputs en s pour avoir une vitesse

        vit_alpha += 10 * sgn_target ;
    }
    else {
        vit_alpha -= 10 * sgn_cpt;
    }

    // Justifier l'utilisation de target ou cpt selon la différence avec le tableau.

}

void asserve_elbow(){

    int sgn_target = signof(robot->get_beta_diff());
    int sgn_cpt = signof(cpt_elbow_vit);

    if (sgn_cpt * cpt_elbow_vit < sgn_target* robot->get_beta_diff()*( 1000 / ( serial_loop_duration ) )){
        
        vit_beta += sgn_target * 10;
    }
    else {
        vit_beta -= sgn_cpt * 10;
    }
}

// Fonctions pour ctiver les moteurs

void activate_shoulder(){

    // Vérification des bornes
    if (vit_alpha>255){
        vit_alpha = 255;
    }
    else if (vit_alpha<-255){
        vit_alpha = -255;
    }
    
    // Activation des moteurs dans le sens correspondant.
    if (vit_alpha > 0){
        digitalWrite(shoulderMotorDir,HIGH);
        analogWrite(shoulderMotorPwm, vit_alpha);
	//rotation vers le haut
        
    }
    else{
        digitalWrite(shoulderMotorDir,LOW);
        analogWrite(shoulderMotorPwm, -vit_alpha);	
	//rotation vers le bas
        
    }
}

void activate_elbow(){

    // Vérification des bornes
    if (vit_beta > 255){
        vit_beta = 255;
    }
    else if (vit_beta < -255){
        vit_beta = -255;
    }

    // Activation des moteurs dans le sens correspondant.
    if (vit_beta > 0){
        digitalWrite(elbowMotorDir,HIGH);
        analogWrite(elbowMotorPwm,vit_beta);
	//rotation vers le haut
        
    }
    else{
        digitalWrite(elbowMotorDir,LOW);
        analogWrite(elbowMotorPwm, -vit_beta);
	//rotation vers le bas
        
    }
}

void activate_rotation(){

    
    if (vit_rotation > 255){
        vit_rotation = 255;
    }
    else if (vit_rotation < -255){
        vit_rotation = -255;
    }
    if (vit_rotation > 0){
        digitalWrite(rotationMotorDir, HIGH);
        analogWrite(rotationMotorPwm, vit_rotation);
        // rotation vers la droite
   

    }
    else{
        digitalWrite(rotationMotorDir, LOW);
        analogWrite(rotationMotorPwm, -vit_rotation);
        // rotation vers la gauche

    }
}

void activate_wrist(int steps){

    stepperMotor->step(steps);
    // Tourne X steps avec une vitesse dépendant de l'input_y. Il faudra synchroniser à la fois la vitesse et le déplacement. 

}


// Fonctions pour activer les capteurs. 

void updateShoulderEncoder()
{
    
 
        if (digitalRead(shoulder_cpt_dir) == HIGH) {
        shoulderEncoderValue++;
	//On augmente la valeur du compteur si on tourne dans le sens direct (indiqué par HIGH)
        }
        else {
        shoulderEncoderValue--;
	//Inversement
        }
        
        
    
  
    
}

void updateElbowEncoder()
{
    
    if (digitalRead(elbow_cpt_dir) == HIGH) {
        elbowEncoderValue++;
    }
    else {
        elbowEncoderValue--;
    }
    
}

void encoderInit()
{
     // Déclenche  une update des capteurs à chaque signal montant détecté.
    attachInterrupt(digitalPinToInterrupt(shoulder_cpt), updateShoulderEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(elbow_cpt), updateElbowEncoder, RISING);

}





void captor_activation(){
    

    
    // variation d'angle : (nombre d'encoches comptées en X secondes / X secondes * 2pi / (nombretotal d'encoches sur un tour)

    // pour le moment, on les calcule en rad  : on a la variation depuis la dernière vérification
    cpt_shoulder_vit = (float)(shoulderEncoderValue  * (2 * 3.141592) / encoder_PPR);
    // en rad

    


    cpt_elbow_vit = (float)(elbowEncoderValue  * (2 * 3.141592) / encoder_PPR);
    
    
    robot->captor_update(cpt_shoulder_vit/40, cpt_elbow_vit/40);
    // On divise par 40, car la vitesse des capteurs est celle du moteur, mais il y a une vis de réduction de 40 entre le moteur et le bras : le bras bouge donc à une vitesse réduite par 40
    // On envoie la modification au robot
    
    float duration_sec = loop_duration/(float)1000;
    
    cpt_elbow_vit = cpt_elbow_vit / duration_sec;
    cpt_shoulder_vit = cpt_shoulder_vit / duration_sec;
    // On divise par la durée du tour ramenée en seconde pour avoir une vitesse, qui servira pour les autres fonctions.
    
    
    shoulderEncoderValue = 0;
    elbowEncoderValue = 0;
    // On réitialise le compte des encodeurs.
    
}// active les capteurs et récupère les données de vitesse de rotation.


// Réinitialise le robot dans une position sûre.
void reset(){

    vit_alpha = 0;
    vit_beta = 0;
    // On réinitialise les vitesses à 0.
    activate_shoulder();
    activate_elbow();
    //On s'asure que les moteurs sont arrêtés.	

    bool stop_shoulder = 0;
    bool stop_elbow = 0;
    // Booléens d'arrêt de boucles.



    loop_duration = 20; 
    //On fixe arbitrairement la durée d'un tour.
    

    // On attend une seconde avant le début de la première phase.
    delay(1000);
    while (stop_elbow==0){

        activate_elbow();
        captor_activation();
        if(cpt_elbow_vit/40 < 3.14159/8){
            vit_beta+=10;

            // Micro asservissement. Si la vitesse DU BRAS (raison pour laquelle on divise la vitesse par 40) est inférieure à pi/8 rad.s-1, on accélère, sinon on ralentit.
            
        }
        else{
            vit_beta -= 10;
        }


        if(vit_beta>=250){
            vit_beta=0;
            activate_elbow();
            stop_elbow=1;
            // Si la demande aux moteurs est trop forte, cela signifie qu'on a atteint un butoir : on arrête les moteurs et on sort de la boucle.
        }
        
 
        delay(20);
    }

    // On attend une autre seconde avant la seconde phase qui réinitialise le bras. On fait l'avant bras avant le bras car comme ça, l'avant bras étant proche du bras, le levier est moins important.
    delay(1000);
    while (stop_shoulder==0){

        activate_shoulder();
        captor_activation();
        if( -cpt_shoulder_vit/40 < 3.14159/8){
            vit_alpha-=10;
            // Micro asservissement. Si la vitesse DU BRAS (raison pour laquelle on divise la vitesse par 40) est inférieure à pi/4 rad.s-1, on accélère, sinon on ralentit.
            // On accélère en diminuant car on cherche une vitesse négative : on tourne vers le bas.
        }
        else{
            vit_alpha += 10;
        }
        // Ce if suppose que la vitesse mesurée est positive stricte !

        if(vit_alpha<=-250){
            vit_alpha=0;
            activate_shoulder();
            stop_shoulder=1;
            // Si la demande aux moteurs est trop forte, cela signifie qu'on a atteint un butoir : on arrête les moteurs et on sort de la boucle.
            // A terme, on pourrait faire ça avec un bouton de fin de butée
        }
        // Si les capteurs détectent une résistance suffisante pour limite arrêter le bras, on baisse fortement la vitesse. 

        // Petite délai entre chaque étape de la boucle

        delay(20);

    }
        
    robot->reset();
    // Réinitialise les données de position interne du robot. 
}




int loop_count = 0;



void setup(){
    Serial.begin(9600); // opens serial port, sets data rate to 9600 bps

    pinMode(shoulderMotorPwm, OUTPUT);
    pinMode(shoulderMotorDir, OUTPUT);
    pinMode(elbowMotorPwm, OUTPUT);
    pinMode(elbowMotorDir, OUTPUT);
    pinMode(rotationMotorPwm, OUTPUT);
    pinMode(rotationMotorDir, OUTPUT);
	// Init des mcc

    pinMode(shoulder_cpt, INPUT);
    pinMode(shoulder_cpt_dir, INPUT);


    pinMode(13, OUTPUT);
    

    
     // Init du Stepper, vitesse de rotation en rpm. 
     stepperMotor->setSpeed(20);

    

    reset();
    //Calibrage
    Serial.print("Reset done.");


    Serial.write("Started.\n");


    
    
    

}

void loop(){


    // Serial.print("Entered loop");

  

    previousMillis = currentMillis;
    currentMillis = millis();
    loop_duration = currentMillis - previousMillis;
    //Calcul de la durée d'un tour.
    
    





    


    


    steps=0;
    // On réinitialise le nombre de steps à donner au stepper.

    //Réception des données serial
    if (Serial.available()){


         buf[cur] = Serial.read();
             if (buf[cur] != -1){
                 cur++;
                 buf[cur] = '\0';
             }
            
            
            
    //         Serial.print(buf[cur - 1]); //debug
        
        // Plan pour la démonstration : on va enregistrer différents messages serial qui seront changés régulièrement afin de créer un mouvement.

        // Button = 0 pour x, button = 1 pour z


        
        if (buf[cur - 1] == '\n' || buf[cur - 1] == '\r') {
            Serial.write("Received buffer : ");
            Serial.write(buf);
            Serial.write("\n");
            validBuf=true;
        }
            
            // Si on a un \n, alors on a un message complet : on vérifie si on est dans un intervalle valide
            
            if (currentMillis - previousMillisSerial > serial_interval && validBuf){

                // Joystick X = 5, Joystick Y = 0, Wrist Angle is 0,  Button is 0, Reset is 0, joystick Max is : 10\n
                // Joystick X = %f, Joystick Y = %f, Wrist Angle is %d,  Button is %d, Reset is %d, joystick Max is : %f


                
            
                // On est dans un intervalle valide, on active la mise à jour des données du robot.
               
                serial_loop_duration = currentMillis - previousMillisSerial;

                previousMillisSerial = currentMillis;


                // Déclenché si on repère qu'on a un serial complet
                int resetDigit = robot->serial_processing(buf);
                // Processing du serial en données utilisables
                
     
                
                // Réinitialisation du buffer.

                robot->input_processing();
                // Processing des données pour obtenir la position voulue et les angles voulus (appelle déjà angle processing).
                // On en déduit alpha, beta et eta cible.


                steps = robot->get_wrist_base_angle() * 360 / (2*3.141592) / step_angle - stepper_base;
                // On ajoute au nombre de steps à effectuer la différence entre la ouvelle position voulue et l'ancienne sauvegardée. Ainsi, si la position ne change pas, on ne donne pas de commande au stepper.

                // stepper_base = robot->get_wrist_base_angle() * 360 / (2*3.141592) / step_angle;
                // Mise à jour du nombre de steps de base.
                validBuf = false;
                cur=0;
                buf[cur]='\0';

                if(resetDigit == 1){
                    reset();
                }

                loop_count++;

                }


        
     }else{
         Serial.print("Serial not available\n");
     }
    
    // A cette étape, on a soit les nouvelles données pour x, y, z et les angles, soit les données précédentes. On peut donc procéder au mouvement.
    

    // Mouvements et asservissement : a lieu seulement tous les asserve_interval (~ 20ms )

    

    // On récupère les données capteurs et on met à jour les données du robot.
    captor_activation();

    // Activation des moteurs avec la puissance donnée :

    activate_shoulder();
    activate_elbow();
    activate_rotation();


    steps -= vit_beta / step_angle * 360 / (2*3.141592);
    activate_wrist(steps);
    // L'angle donné au poignet est l'inverse de celui de l'avant bras. Il suffit de prendre la vitesse donnée à l'avant bras et de la convertir en steps pour avoir le mouvement du poignet pour qu'il reste stable.
    // On fait un += pour si jamais l'angle de base change aussi. En théorie, les deux évènements sont séparés, mais au cas où.

    // Asservissement : 
    // Si la vitesse mesurée est différente de la vitesse voulue (l'angle multiplié par la fréquence d'envoi des commandes), on modifie légèrement l'ordre donné aux moteurs
    asserve_shoulder();
    asserve_elbow();



}

