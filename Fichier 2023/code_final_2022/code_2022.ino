#include <Arduino.h>
#include <Servo.h>
#include <Stepper.h>
#include "arm.h"
#include <stdio.h>


// // #DEFINE ENCODEROUTPUT 64*4


// Constants 


// Déclaration des ports capteurs.
const int shoulder_cpt = 18;
const int shoulder_cpt_dir = 19;
const int elbow_cpt = 20;
const int elbow_cpt_dir = 21;
// Le port sans nom est le premier port de réception. Le port _2 correspond à l'input donnant le sens de rotation. 


volatile int shoulderEncoderValue = 0; 
volatile int elbowEncoderValue = 0;
// Compteurs
float cpt_shoulder_vit = 0;
float cpt_elbow_vit = 0;
const int encoder_PPR = 2803/4;



//  ############# IMPORTANT #############
// Le programme tourne sans doute trop vite ! On peut réduire l'acquisition des données seriales à 30Hz (33ms) et l'asservissement à 300 Hz (3.3 ms).
// Ou meme à 10 Hz pour les données serial et 50Hz pour l'asservissement  



// Déclaration des capteurs et des données :
// Ports, valeur du capteur, et stockage des vitesses en rad.s-1
// L'encodeur a un Count per Revolution de 64 avant réduction, et de 2803 après réduction,  donc un Pulse per Revolution de 4*2803.


const float arm = 25;
const float foreArm = 25;
const float wrist = 10;
// Déclaration des tailles des membres, en cm.
 

// const float stepper_modifier = 1.5;
// Modificateur pour transformer l'input
const int number_steps = 200;
const int step_angle = 360 / number_steps; // = 1.8 degrés, à combiner avec le réducteur de 40.
int steps = 0; //Nombre de steps qu'on demandera au stepper de bouger. 
int stepper_base = 0; //Sauvegarde de la position du stepper. utilisé lorsque des ordres sont reçus. 


// const int Tour=0; // Déclaration variable nombre de tour
// const int delayTime=8; // Déclaration variable vitesse du moteur


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
// Vitesses/puissances demandées aux moteurs des angles alpha et beta.
// int pow_stepper = 0;
// int old_pow_stepper = 0;
// // ? Inutiles




//variables de temps pour les capteurs
const long asserve_interval = 20000;
// en microsecondes. Temps minimal d'activation entre deux capteurs.
const long serial_interval = 100000;
// en microsecondes, soit 100ms, soit 1/10 secondes. Temps minimal d'espacement entre deux réceptions de données.
// A synchroniser avec le fichier arm.cpp



long loop_duration = 1;
long serial_loop_duration = 1;
// temps s'étant réellement écoulé en us entre les deux activations des capteurs et entre deux réceptions serial. Variable.

long previousMicros = 0;
long currentMicros = 0;
// long currentMicrosSerial = 0;
long previousMicrosSerial = 0;
// pour compter le temps

// Déclaration des MCC
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



// Fonctions pour asservir les moteurs
void asserve_shoulder(){

    int sgn_target = signof(robot->get_alpha_diff());
    // sgn = -1 si l'angle cible est inférieur à l'angle souhaité, et inversement
    int sgn_cpt = signof(cpt_shoulder_vit);

    // On compare les valeurs obsolues
    if (sgn_cpt*(cpt_shoulder_vit) < sgn_target * robot->get_alpha_diff() *  ( 1000000 / ( serial_loop_duration ) ) ){
        // Si la vitesse de rotation, en radians en valeur absolue, est inférieure à la valeur absolue de la différence d'angles divisé par la période (ce qui donne une vitesse de rotation)
        // *1000000/serial_interval car (robot->alpha - robot->alpha_p) est un angle, on divise par la période d'acquisition des inputs en s pour avoir une vitesse

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

    if (sgn_cpt * cpt_elbow_vit < sgn_target* robot->get_beta_diff()*( 1000000 / ( serial_loop_duration ) )){
        
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
        
    }
    else{
        digitalWrite(shoulderMotorDir,LOW);
        analogWrite(shoulderMotorPwm, -vit_alpha);
        
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
        
    }
    else{
        digitalWrite(elbowMotorDir,LOW);
        analogWrite(elbowMotorPwm, -vit_beta);
        
    }
}

void activate_rotation(){

    // vit_rotation = +200;
    if (vit_rotation > 255){
        vit_rotation = 255;
    }
    else if (vit_rotation < -255){
        vit_rotation = -255;
    }
    if (vit_rotation > 0){
        digitalWrite(rotationMotorDir, HIGH);
        analogWrite(rotationMotorPwm, vit_rotation);
        // Serial.print(vit_rotation);
        // DROITE
        // Serial.print("Turning in normal direction\n");

    }
    else{
        digitalWrite(rotationMotorDir, LOW);
        analogWrite(rotationMotorPwm, -vit_rotation);
        // GAUCHE
        // Serial.print(- vit_rotation);
        // Serial.print("Turning in opposite direction\n");
    }
}

void activate_wrist(int steps){

   
    stepperMotor->step(steps);
    // Tourne X steps avec une vitesse dépendant de l'input_y. Il faudra synchroniser à la fois la vitesse et le déplacement. On ne peut pas asservir cette partie car le stepper n'est pas asservi !

}


// Fonctions pour activer les capteurs. Tels qu'ils sont codés, les capteurs donnent une vitesse en valeur absolue.
// Il serait potentiellement profitable de les coder en position.

void updateShoulderEncoder()
{
    
    // Modification pour un capteur fonctionnant en position : 
    // Avec ces modifications, la valeur de shoulderEncoderValue donnera une valeur avec offset de l'angle du robot et donc de sa position précise.
 
        if (digitalRead(shoulder_cpt_dir) == HIGH) {
        shoulderEncoderValue++;
        }
        else {
        shoulderEncoderValue--;
        }
        // Serial.print("Called udpateShoulderEncoder\n");
        
    
  
    
}

void updateElbowEncoder()
{
    
    if (digitalRead(elbow_cpt_dir) == HIGH) {
        elbowEncoderValue++;
    }
    else {
        elbowEncoderValue--;
    }
    // Serial.print("Called updateElbowEncoder\n");
}

void encoderInit()
{
     // Déclenche  une update des capteurs à chaque signal montant détecté.
    attachInterrupt(digitalPinToInterrupt(shoulder_cpt), updateShoulderEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(elbow_cpt), updateElbowEncoder, RISING);

    // NOTE : pour le sens de rotation, bien faire attention à si on regarde montant ou descendant par rapport à si on regarde High or Low sur le capteur secondaire.
}





void captor_activation(){
    

    // vitesse de rotation =
    // (nombre d'encoches comptées en X secondes / X secondes * 2pi / (nombretotal d'encoches sur un tour)

    // pour le moment, on les calcule en rad  : on a la variation depuis la dernière vérification
    cpt_shoulder_vit = (float)(shoulderEncoderValue  * (2 * 3.141592) / encoder_PPR);
    // en rad

    // cpt_shoulder_vit =(float) shoulderEncoderValue  * (360 / encoder_PPR) * 2*3.141592 / 360;


    cpt_elbow_vit = (float)(elbowEncoderValue  * (2 * 3.141592) / encoder_PPR);
    
    //  /!\ PRENDRE EN COMPTE LE REDUCTEUR AJOUTE DE 40 POUR LE CAPTOR UPDATE !
    robot->captor_update(cpt_shoulder_vit/40, cpt_elbow_vit/40);
    // On divise par 40, car la vitesse des capteurs est celle du moteur, mais il y a une vis de réduction de 40 entre le moteur et le bras : le bras bouge donc à une vitesse réduite par 40
    // On envoie la modification au robot
    
    float duration_sec = loop_duration/(float)1000000;
    
    cpt_elbow_vit = cpt_elbow_vit / duration_sec;
    cpt_shoulder_vit = cpt_shoulder_vit / duration_sec;
    // On divise par la durée du tour ramenée en seconde pour avoir une vitesse, qui servira pour les autres fonctions.


    // Only update display when there have readings
    // if ( rpm > 0) {
    // Serial.print(encoderValue);
    // Serial.print(" pulse / ");
    // Serial.print(ENCODEROUTPUT);
    // Serial.print(" pulse per rotation x 60 seconds = ");
    // Serial.print(rpm);
    // Serial.println(" RPM");
    // // }

    // Serial.print("Loop Duration : ");
    // Serial.print(loop_duration);
    // Serial.print(" || duration sec : ");
    // Serial.print(duration_sec);

    // Serial.print(" ||Vitesse de l'épaule : ");
    // Serial.print(cpt_shoulder_vit);
    // Serial.print(", nombre d'encodages :");
    // Serial.print(shoulderEncoderValue);
    // Serial.write(" || ");

    // Serial.print("Vitesse du coude : ");
    // Serial.print(cpt_elbow_vit);
    // Serial.print(", nombre d'encodages :");
    // Serial.print(elbowEncoderValue);
    // Serial.write("\n");
    
    
    shoulderEncoderValue = 0;
    elbowEncoderValue = 0;
    // On rénitialise le compte des encodeurs.
    
}// active les capteurs et récupère les données de vitesse de rotation.

// Réinitialise le robot dans une position sûre.
void reset(){

    vit_alpha = 0;
    vit_beta = 0;
    // On réinitialise les vitesses à 0.
    activate_shoulder();
    activate_elbow();

    bool stop_shoulder = 0;
    bool stop_elbow = 0;
    // Booléens d'arrêt de boucles.

    // On attend une seconde avant le début de la première phase.

    loop_duration = 20000;
    

    
    delay(1000);
    while (stop_elbow==0){

        activate_elbow();
        captor_activation();
        if(cpt_elbow_vit/40 < 3.14159/8){
            vit_beta+=10;

            // Micro asservissement. Si la vitesse DU BRAS (raison pour laquelle on divise la vitesse par 40) est inférieure à pi/4 rad.s-1, on accélère, sinon on ralentit.
            
        }
        else{
            vit_beta -= 10;
        }
        // Ce if suppose que la vitesse mesurée est positive stricte ! Et que du coup elle est dans le bon sens... 

        if(vit_beta>=250){
            vit_beta=0;
            activate_elbow();
            stop_elbow=1;
            // Si la demande aux moteurs est trop forte, cela signifie qu'on a atteint un butoir : on arrête les moteurs et on sort de la boucle.
        }
        // Si les capteurs détectent une résistance suffisante pour limite arrêter le bras, on baisse fortement la vitesse.
        
        // Serial.print("Vit beta : ");
        // Serial.print(vit_beta);
        // Serial.print(" || ");
        delay(20);
    }

    // On attend une autre seconde avant la seconde phase qui réinitialise le bras. On f'ait l'avant bras avant le bras car comme ça, l'avant bras étant proche du bras, le levier est moins important.
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
        Serial.print("Vit alpha : ");
        Serial.print(vit_alpha);
        Serial.print(" ||\n ");
        delay(20);

    }
        
    robot->reset();
    // Réinitialise les données de position interne du robot. 
}

size_t strlcpy(char *dst, const char *src, size_t size)
{
    size_t len = 0;
    while(size > 1 && *src)
    {
        *dst++ = *src++;
        size--;
        len++;
    }
    if(size > 0)
        *dst = 0;
    return len + strlen(src);    
};
// Fonction courte peremtant de mettre un char const dans un char[]


int loop_count = 0;



void setup(){
    Serial.begin(9600); // opens serial port, sets data rate to 9600 bps


    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    
    // Affectation des pins des moteurs et mise à 0 pour empêcher le courant de passer dans les moteurs
    
    pinMode(shoulderMotorPwm, OUTPUT);
    digitalWrite(shoulderMotorPwm, LOW);
    
    pinMode(shoulderMotorDir, OUTPUT);
    digitalWrite(shoulderMotorDir, LOW);

    pinMode(elbowMotorPwm, OUTPUT);
    digitalWrite(elbowMotorPwm, LOW);
    
    pinMode(elbowMotorDir, OUTPUT);
    digitalWrite(elbowMotorDir, LOW);
    
    pinMode(rotationMotorPwm, OUTPUT);
    digitalWrite(rotationMotorPwm, LOW);

    pinMode(rotationMotorDir, OUTPUT);
    digitalWrite(rotationMotorDir, LOW);

    pinMode(shoulder_cpt, INPUT);
    pinMode(shoulder_cpt_dir, INPUT);
    // attachInterrupt(digitalPinToInterrupt(shoulder_cpt), updateShoulderEncoder, RISING);
    encoderInit();
    

    
    // // Init du Stepper, vitesse de rotation en rpm. 
    // stepperMotor->setSpeed(20);

    // encoderInit();
    // // Initialise les encodeurs

    // vit_alpha = 100;

    delay(3000);
    // vit_beta = 100;
    // // captor_activation();
    // activate_elbow();
    
    // delay(1000);
    // vit_alpha = 0;
    // vit_beta = 0;
    // loop_duration = 1000;
    // captor_activation();
    // activate_elbow();

    

    // delay(50000);
    Serial.print("Starting going up before setup\n");
    
    vit_beta= -50;
    activate_elbow();
    delay(10000);
    vit_beta = 0;
    activate_elbow();  

    delay(4000);
    
    vit_alpha = -200;
    activate_shoulder();
    delay(1800);
    vit_alpha = 0;
    activate_shoulder();



    
    Serial.print("Done. Starting Reset.\n");

   delay(1000000);
    reset();
    Serial.print("Reset done.");

 

    

    Serial.write("Started.\n");


    
    
    

}

void loop(){


//    // Serial.print("Entered loop");
//
//    // digitalWrite(13, HIGH);
//    // delay(1000);
//    // digitalWrite(13, LOW);
//    // delay(1000);
//    // Serial.write("Looping...");
//
//    previousMicros = currentMicros;
//    currentMicros = micros();
//    loop_duration = currentMicros - previousMicros;
//    
//    // Serial.write("Loop Duration : ");
//    // Serial.println(loop_duration / 1000);
//    // Serial.write(" ms\n");
//    // True interval désigne le temps du tour précédent. 
//    // Problème potentiel : la fonction micors() boucle après 70 minutes. Pour une boucle toutes les 70 minutes, loop_duration aura une valeur négative très élevée.
//
//    // Initialisation
//    // digitalWrite(enableBridge1,HIGH); // Active pont en H
//    // digitalWrite(enableBridge2,HIGH); // Active pont en H
//
//    // Ce commentaire est totalement inutile.
//
//    
//
//
//    
//
//
//    steps=0;
//    // On réinitialise le nombre de steps à donner au stepper.
//
//    // Réception des données serial
//    // if (Serial.available()){
//
//
//    //     buf[cur] = Serial.read();
//    //         if (buf[cur] != -1){
//    //             cur++;
//    //             buf[cur] = '\0';
//    //         }
//            
//            
//            
//    //         Serial.print(buf[cur - 1]); //debug
//        
//        // Plan pour la démonstration : on va enregistrer différents messages serial qui seront changés régulièrement afin de créer un mouvement.
//
//        // Button = 0 pour x, button = 1 pour z
//
//        if (loop_count<2000){
//            strlcpy(buf, "Joystick X = 5, Joystick Y = 0, Wrist Angle is 0,  Button is 0, Reset is 0, joystick Max is : 10\n", 1024);
//            
//            validBuf=true;
//        }
//        else if (loop_count<30){
//            strlcpy(buf, "Joystick X = 5, Joystick Y = 5, Wrist Angle is 0,  Button is 1, Reset is 0, joystick Max is : 10\n", 1024);
//            validBuf=true;
//        }
//        else if (loop_count<40){
//            strlcpy(buf, "Joystick X = 5, Joystick Y = 0, Wrist Angle is 0,  Button is 1, Reset is 0, joystick Max is : 10\n", 1024);
//            validBuf=true;
//        }
//        else if (loop_count<50){
//            strlcpy(buf, "Joystick X = 5, Joystick Y = 0, Wrist Angle is 0,  Button is 0, Reset is 0, joystick Max is : 10\n", 1024);
//            validBuf=true;
//        }
//        else if (loop_count<55){
//            strlcpy(buf, "Joystick X = -5, Joystick Y = 0, Wrist Angle is 0,  Button is 0, Reset is 0, joystick Max is : 10\n", 1024);
//            validBuf=true;
//        }
//        else if (loop_count<70){
//            strlcpy(buf, "Joystick X = -5, Joystick Y = -7, Wrist Angle is 0,  Button is 1, Reset is 0, joystick Max is : 10\n", 1024);
//            validBuf=true;
//        }
//        else if (loop_count<70){
//            strlcpy(buf, "Joystick X = -5, Joystick Y = -7, Wrist Angle is 0,  Button is 1, Reset is 0, joystick Max is : 10\n", 1024);
//            validBuf=true;
//        }
//        else {
//            strlcpy(buf, "Joystick X = 0, Joystick Y = 0, Wrist Angle is 0,  Button is 0, Reset is 1, joystick Max is : 10\n", 1024);
//            validBuf = true;
//            loop_count = 0;
//        }
//        Serial.print("Valeur du loop : ");
//        Serial.print(loop_count);
//        Serial.print("\n");
//
//
//        
//        if (buf[cur - 1] == '\n' || buf[cur - 1] == '\r') {
//            Serial.write("Received buffer : ");
//            Serial.write(buf);
//            Serial.write("\n");
//            validBuf=true;
//        }
//            
//            // Si on a un \n, alors on a un message complet : on vérifie si on est dans un intervalle valide
//            
//            if (currentMicros - previousMicrosSerial > serial_interval && validBuf){
//
//                // Joystick X = 5, Joystick Y = 0, Wrist Angle is 0,  Button is 0, Reset is 0, joystick Max is : 10\n
//                // Joystick X = %f, Joystick Y = %f, Wrist Angle is %d,  Button is %d, Reset is %d, joystick Max is : %f
//
//
//                Serial.write("Entered buffer treatement\n");
//            
//                // On est dans un intervalle valide, on active la mise à jour des données du robot.
//                Serial.write("Treatement buffer interval : ");
//                Serial.println((currentMicros - previousMicrosSerial)/1000);
//                Serial.write("\n");
//                serial_loop_duration = currentMicros - previousMicrosSerial;
//
//                previousMicrosSerial = currentMicros;
//
//
//                // Déclenché si on repère qu'on a un serial complet
//                int resetDigit = robot->serial_processing(buf);
//                // Processing du serial en données utilisables
//                
//
//                
//
//               
//                
//                
//                // Réinitialisation du buffer.
//
//                robot->input_processing();
//                // Processing des données pour obtenir la position voulue et les angles voulus (appelle déjà angle processing).
//                // On en déduit alpha, beta et eta cible.
//
//
//                steps = robot->get_wrist_base_angle() * 360 / (2*3.141592) / step_angle - stepper_base;
//                // On ajoute au nombre de steps à effectuer la différence entre la ouvelle position voulue et l'ancienne sauvegardée. Ainsi, si la position ne change pas, on ne donne pas de commande au stepper.
//
//                // stepper_base = robot->get_wrist_base_angle() * 360 / (2*3.141592) / step_angle;
//                // Mise à jour du nombre de steps de base.
//                validBuf = false;
//                cur=0;
//                buf[cur]='\0';
//
//                if(resetDigit == 1){
//                    reset();
//                }
//
//                loop_count++;
//
//                }
//
//
//            // buf[0]='\0';
//            // cur=0;
//            // Serial.write("Deleted Buf\n");
//            // On clear le buffer dès qu'il est plein pour ne pas rater les prochaines infos, qu'on les aie utilisées ou non.
//
//            
//        
//        
//    // }else{
//    //     Serial.print("Serial not available\n");
//    // }
//    
//    // A cette étape, on a soit les nouvelles données pour x, y, z et les angles, soit les données précédentes. On peut donc procéder au mouvement.
//    
//
//    // Mouvements et asservissement : a lieu seulement tous les asserve_interval (~ 20ms )
//
//    
//
//    // On récupère les données capteurs et on met à jour les données du robot.
//    captor_activation();
//
//    Serial.print("Activation de l'asservissement, loop_duration : ");
//    Serial.print(loop_duration/1000);
//    Serial.print(" ms\n");
//
//    // Activation des moteurs avec la puissance donnée :
//
//    activate_shoulder();
//    activate_elbow();
//    activate_rotation();
//
//
//    steps += vit_beta / step_angle * 360 / (2*3.141592);
//    // activate_wrist(steps);
//    // L'angle donné au poignet est l'inverse de celui de l'avant bras. Il suffit de prendre la vitesse donnée à l'avant bras et de la convertir en steps pour avoir le mouvement du poignet pour qu'il reste stable.
//    // On fait un += pour si jamais l'angle de base change aussi. En théorie, les deux évènements sont séparés, mais au cas où.
//
//    // Asservissement : 
//    // Si la vitesse mesurée est différente de la vitesse voulue (l'angle multiplié par la fréquence d'envoi des commandes), on modifie légèrement l'ordre donné aux moteurs
//    asserve_shoulder();
//    asserve_elbow();
//
//    

    

    


}
