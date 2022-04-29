/******************************************************************************
Asservissement de vitesse d'un moteur à courant continu
Ce programme est destiné à fonctionner avec l'ensemble de 
commande de moteur à courant continu, disponible à l'adresse suivante:
http://boutique.3sigma.fr/robots/23-experience-commande-de-moteur-electrique.html

Auteur: 3Sigma
Version 3.1 - 24/01/2017
*******************************************************************************/


// Inclusion d'une bibliothèque permettant l'exécution à cadence fixe
// d'une partie du programme. Télécharger à l'adresse http://www.3sigma.fr/telechargements/FlexiTimer2.zip
// et décompresser dans le sous-répertoire « libraries » de votre installation Arduino
// Pour plus de détails, voir les pages (en anglais): 
// http://www.arduino.cc/playground/Main/FlexiTimer2
// https://github.com/wimleers/flexitimer2
#include <FlexiTimer2.h>

// Inclusion d'une bibliothèque permettant de lire et d'écrire plus rapidement sur les entrées-sorties digitales.
// Télécharger à l'adresse http://www.3sigma.fr/telechargements/digitalWriteFast.zip
// et décompresser dans le sous-répertoire « libraries » de votre installation Arduino
#include <digitalWriteFast.h> 

// Définitions et déclarations pour le codeur incrémental
#define codeurInterruptionA 0
#define codeurInterruptionB 1
#define codeurPinA 2
#define codeurPinB 3
#define Nmoy 10
volatile long ticksCodeur = 0;
static int indiceTicksCodeur = 0;
static int ticksCodeurTab[Nmoy];
static int codeurDeltaPos;

// Définitions et déclarations pour le moteur à courant continu.
#define directionMoteur  4
#define pwmMoteur  5
#define mesurePWM  8
// ATTENTION: donner la bonne valeur de la tension d'alimentation sur la ligne ci-dessous. Ici, 9V
double tensionAlim = 9.;

// Ce programme reçoit par liaison série des données issues d'un programme qui est exécuté sur
// l'ordinateur connecté à la carte. Si le programme ne reçoit rien pendant un temps donné,
// défini ci-dessous, un traitement particulier sera réalisé.
// Timeout de réception de données en s
#define TIMEOUT 2
unsigned long timeLastReceived = 0;
static int timedOut = 0;

// Ce programme envoie par liaison série des données à l'ordinateur connecté à la carte.
// Cadence d'envoi des données en ms
#define TSDATA 20
unsigned long tempsDernierEnvoi = 0;
unsigned long tempsCourant = 0;

// Certaines parties du programme sont exécutées à cadence fixe grâce à la bibliothèque FlexiTimer2.
// Cadence d'échantillonnage en ms
#define CADENCE_MS 10
volatile double dt = CADENCE_MS/1000.;
volatile double temps = -CADENCE_MS/1000.;

// Le moteur est asservi en vitesse grâce à un régulateur de type PID
// On déclare ci-dessous les variables et paramètres nécessaires à l'asservissement et au régulateur
static double vref = 0.; // vitesse de consigne
static double omega = 0.; // vitesse de rotation du moteur
static double commande = 0.; // commande en tension calculée par le PID
static double commande_avant_sat = 0.; // valeur de la commande avant la saturation (voir ci-dessous)
static double umax = 6.; // valeur max de la tension de commande du moteur
static double umin = -6.; // valeur min (ou max en négatif) de la tension de commande du moteur
static double yprec = 0; // mesure de la vitesse au calcul précédent
static double Tf = 0.02; // constante de temps de filtrage de l'action dérivée du PID
static double P_x = 0.; // valeur de l'action proportionnelle
static double I_x = 0.; // valeur de l'action intégrale
static double D_x = 0.; // valeur de l'action dérivée
// Variables intermédiaires
static double Ti = 0;
static double Td = 0;
static double Tt = 0;
static double ad = 0;
static double bd = 0;
static double br = 0;

// Déclaration des variables pour la réception des données sur la ligne série
const int NOMBRE_DE_CHAMPS = 8; // nombre de champs de données reçues sur la liaison série
int champIndex = 0; // champ courant reçu
int valeursRecues[NOMBRE_DE_CHAMPS]; // tableau contenant les champs de données reçues
// Variables utilisées pour les données reçues
static int typeSignal = 0;
static double offset = 0.;
static double amplitude = 0.;
static double frequence = 0.;
static double Kp = 0.25; // gain proportionnel du PID
static double Ki = 1.5; // gain intégral du PID
static double Kd = 0.0; // gain dérivé du PID
// Variables pour la vérification de cohérence des données
static double crc = 0.;
static double crc_check = 0.;

// Initialisations
void setup(void) {
  
  // Codeur incrémental
  pinMode(codeurPinA, INPUT);      // entrée digitale pin A codeur
  pinMode(codeurPinB, INPUT);      // entrée digitale pin B codeur
  digitalWrite(codeurPinA, HIGH);  // activation de la résistance de pullup
  digitalWrite(codeurPinB, HIGH);  // activation de la résistance de pullup
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurPinA (définie à la fin du programme)
  attachInterrupt(codeurInterruptionA, GestionInterruptionCodeurPinA, CHANGE);
  // A chaque changement de niveau de tension sur le pin B du codeur,
  // on exécute la fonction GestionInterruptionCodeurPinB (définie à la fin du programme)
  attachInterrupt(codeurInterruptionB, GestionInterruptionCodeurPinB, CHANGE);

  // Moteur à courant continu
  pinMode(directionMoteur, OUTPUT);
  pinMode(pwmMoteur, OUTPUT);
  pinMode(mesurePWM, INPUT);

  // Liaison série.
  // ATTENTION: ne pas modifier la vitesse de transmission de la liaison série,
  // sinon le programme de pilotage exécuté sur le PC connecté à la carte ne fonctionnera plus
  Serial.begin(115200);
  Serial.flush();
  
  /* Le programme principal est constitué:
     - de la traditionnelle boucle "loop" des programmes Arduino, dans laquelle on lit 
       en permanence la liaison série pour récupérer les nouvelles consignes de tension
       de commande du moteur
     - de la fonction "isrt", dont l'exécution est définie à cadence fixe par les deux instructions
       ci-dessous. La bonne méthode pour exécuter une fonction à cadence fixe, c'est de l'exécuter sur interruption
       (la boucle "loop" est alors interrompue régulièrement, à la cadence voulue, par un timer).
       Une mauvaise méthode serait d'utiliser la fonction Arduino "delay". En effet, la fonction "delay" définit
       un délai entre la fin des instructions qui la précèdent et le début des instructions qui la suivent, mais si
       on ne connait pas le temps d'exécution de ces instructions (ou si celui-ci est inconnu), on n'obtiendra
       jamais une exécution à cadence fixe.
       Il est nécessaire d'exécuter la fonction "isrt" à cadence fixe car cette fonction:
       - doit renvoyer des données à cadence fixe au programme exécuté sur l'ordinateur connecté à la carte
       - calcule la vitesse de rotation du moteur en comptant le nombre d'impulsions du codeur pendant un temps fixe
     
  */
  FlexiTimer2::set(CADENCE_MS, 1/1000., isrt); // résolution timer = 1 ms
  FlexiTimer2::start();

}


// Boucle principale
void loop() {

  // Réception des données sur la liaison série
  readData();
  
}

// Fonction excutée sur interruption
void isrt(){

  // On calcule une moyenne glissante de la vitesse sur les Nmoy derniers échantillons
  // Nombre de ticks codeur depuis la dernière fois accumulé pour les 1à derniers échantillons
  // Ce nombre est mis à jour par les fonctions GestionInterruptionCodeurPinA et GestionInterruptionCodeurPinB,
  // exécutées à chaque interruption due à une impulsion sur la voie A ou B du codeur incrémental
  for (int i=0; i<Nmoy; i++) {
    ticksCodeurTab[i] += ticksCodeur;
  }  
  // Une fois lu, ce nombre est remis à 0 pour pouvoir l'incrémenter de nouveau sans risquer de débordement de variable
  ticksCodeur = 0;
  
  // Pour l'échantillon courant, calcule de l'angle de rotation du moteur pendant la période d'échantillonnage
  codeurDeltaPos = ticksCodeurTab[indiceTicksCodeur];
  // Remise à zéro du compteur d'impulsion codeur pour l'échantillon courant
  ticksCodeurTab[indiceTicksCodeur] = 0;
  
  // Mise à jour de l'indice d'échantillon courant
  indiceTicksCodeur++;
  if (indiceTicksCodeur==Nmoy) {
    indiceTicksCodeur = 0;
  }

  // Calcul de la vitesse de rotation. C'est le nombre d'impulsions converti en radian, divisé par la période d'échantillonnage
  // On fait un calcul glissant sur Nmoy échantillons, d'où le Nmoy*dt
  omega = ((2.*3.141592*((double)codeurDeltaPos))/1632.)/(Nmoy*dt);  // en rad/s
  
  // Calcul de la consigne en fonction des données reçues sur la liaison série
  if (typeSignal==0) { // signal carré
    if (frequence > 0) {
      if (temps - ((double)((int)(temps*frequence)))/frequence < 1/(2*frequence)) {
        vref = offset + amplitude;
      }
      else {
        vref = offset;
      }
    }
    else {
      vref = offset + amplitude;
    }
  }
  else { // sinus
    if (frequence > 0) {
      vref = offset + amplitude * sin(2*3.141592*frequence*temps);
    }
    else {
      vref = offset + amplitude;
    }
  }
  
  /******* Calcul du PID *******/
  // Paramètres intermédiaires
  Ti = Ki/(Kp+0.01);
  if (Kd>0) { // Si PID
    ad = Tf/(Tf+dt);
    bd = Kd/(Tf+dt);
    Td = Kp/Kd;
    Tt = sqrt(Ti*Td);
  }
  else { // Si PI
    Tt = 0.5*Ti;
  }
  br = dt/(Tt+0.01);
  
  // Terme proportionnel
  P_x = Kp * (vref - omega);
  
  // Terme dérivé
  D_x = ad * D_x - bd * (omega - yprec);
  
  // Calcul de la commande avant saturation
  commande_avant_sat = P_x + I_x + D_x;
  
  // Application de la saturation sur la commande
  if (commande_avant_sat > umax) {
    commande = umax;
  }
  else if (commande_avant_sat < umin) {
    commande = umin;
  }
  else {
    commande = commande_avant_sat;
  }
  
  // Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
  I_x = I_x + Ki * dt * (vref - omega) + br * (commande - commande_avant_sat);
  
  // Stockage de la mesure courant pour utilisation lors du pas d'échantillonnage suivant
  yprec = omega;

  /******* Fin Calcul du PID *******/
      
  // Envoi de la commande au moteur
  // Si le timeout de réception des données a été déclenché (voir plus loin),
  // on met la commande à 0
  if (timedOut==1) {
    commande = 0.;
  }
  CommandeMoteur(commande, tensionAlim);

  // Incrémentation du temps courant
  temps += dt;
}

void ecritureData(void) {
  // Ecriture des données en sortie tous les TSDATA millisecondes
  
  // On envoie les données s'il s'est écoulé plus de TSDATA secondes
  // entre l'instant courant et le dernier envoi
  tempsCourant = millis();
  if (tempsCourant-tempsDernierEnvoi > TSDATA) {
    Serial.print(temps);
    
    Serial.print(",");
    Serial.print(vref);
    Serial.print(",");
    Serial.print(omega);
    Serial.print(",");
    Serial.print(commande);
   
    Serial.print("\r");
    Serial.print("\n");
    
    tempsDernierEnvoi = tempsCourant;
  }  
}

void readData(void) {
  // Lecture des données sur la liaison série
  // On attend une série de NOMBRE_DE_CHAMPS valeurs entières séparées par des virgules
  
  // Initialisations
  champIndex = 0;
  for(int i=0; i < NOMBRE_DE_CHAMPS; i++) {
    valeursRecues[i] = 0;
  }

  while(true) {

    // Ecriture des données sur la liaison série
    ecritureData();
  
    if (Serial.available()>0) { // si une donnée est reçue
      // Réinitialisation du timeout (voir plus loin)
      timeLastReceived = millis();
      timedOut = 0;
      // Lecture de la donnée
      char ch = Serial.read();
      if (ch >= '0' && ch <= '9') { // caractère ascii entre 0 et 9 ?
        // Si oui, nous stockons la valeur
        valeursRecues[champIndex] = (valeursRecues[champIndex] * 10) + (ch - '0');
      }
      else if (ch == ',') { // séparateur virgule détecté, on passe au champ suivant
        if(champIndex < NOMBRE_DE_CHAMPS-1) {
          champIndex++; // incrémentation de l'index de champ
        }
      }
      else {
        // tout caractère autre qu'un chiffre ou une virgule (le retour chariot de fin de trame par exemple)
        // stoppe l'acquisition des champs
        break; // on sort de la boucle
      }
      
    }
    // Timeout
    // Si aucune donnée n'a été reçue pendant plus de TIMEOUT ms, on met à zéro toutes les
    // consignes par sécurité
    else {      
      if ((millis()-timeLastReceived > TIMEOUT*1000) && (timedOut==0)) {
        for(int i=0; i < NOMBRE_DE_CHAMPS; i++) {
          offset = 0.;
          amplitude = 0.;
          frequence = 0.;
        }
        timedOut = 1;
      }
    }    
  }
  
  // Traitement des données reçues (on extrait chaque valeur de la trame complète)
  if (champIndex == NOMBRE_DE_CHAMPS-1) { // si le bon nombre de champs a été reçu
      
    // Récupération de la dernière valeur (somme de contrôle, envoyée par l'émetteur)
    crc = valeursRecues[NOMBRE_DE_CHAMPS-1];
    // Calcul de la somme de contrôle (somme de toutes les autres données) côté récepteur
    crc_check = 0;
    for(int i=0; i < NOMBRE_DE_CHAMPS-1; i++) {
      crc_check += valeursRecues[i];
    }
    
    // Si les sommes de contrôle sont identiques, on récupère les données
    if (crc == crc_check) {
      typeSignal = valeursRecues[0];
      // Conversion des données d'entiers en flottants
      offset = ((double)valeursRecues[1]-100)/10.;
      amplitude = ((double)valeursRecues[2])/10.;
      frequence = ((double)valeursRecues[3])/100.;
      Kp = ((double)valeursRecues[4])/100.;
      Ki = ((double)valeursRecues[5])/10.;
      Kd = ((double)valeursRecues[6])/1000.;
    }
    // Si les sommes de contrôle sont différentes, on fait un traitement particulier
    else {
      // Par exemple, envoi d'un message, désactivé ici
      //Serial.print("\r\r\rErreur CRC !!!");
    }
  }
  else { // on n'a pas reçu le bon nombre de champs
    // On fait un traitement particuliern pas exemple, envoi d'un message (désactivé ici)
    //Serial.print("\r\r\rErreur reception !!!\r\r\r");
  }
}

void CommandeMoteur(double commande, double tensionAlim) {
  // Cette fonction calcule et envoi les signaux PWM au pont en H
  // en fonction des tensions de commande et d'alimentation
  int tension_int;
  double tension;
  
  // L'ensemble pont en H + moteur n'est pas linéaire
  // On redresse la caractéristique avec cette fonction
  tension = redresse_commande(commande, tensionAlim);
  
  // Normalisation de la tension d'alimentation par
  // rapport à la tension d'alimentation
  tension_int = (int)(255*(tension/tensionAlim));
  
  // Saturation par sécurité
  if (tension_int>255) {
    tension_int = 255;
  }
  if (tension_int<-255) {
    tension_int = -255;
  }
  
  // Commande PWM
  if (tension_int>=0) {
    digitalWrite(directionMoteur, LOW);
    analogWrite(pwmMoteur, tension_int);
  }
  if (tension_int<0) {
    digitalWrite(directionMoteur, HIGH);
    analogWrite(pwmMoteur, -tension_int);
  }
}

double redresse_commande(double commande, double tensionAlim) {
  // Cette fonction redresse la non-linéarité de l'ensemble pont en H + moteur
  if (commande>0) {
    return(commande + 0.*tensionAlim/tensionAlim); 
  }
  else if (commande<0) {
    return(commande - 0.*tensionAlim/tensionAlim); 
  }
  else {
    return(0); 
  }
}

 
void GestionInterruptionCodeurPinA() {  
  // Routine de service d'interruption attachée à la voie A du codeur incrémental
  // On utilise la fonction digitalReadFast2 de la librairie digitalWriteFast
  // car les lectures doivent être très rapides pour passer le moins de temps possible
  // dans cette fonction (appelée de nombreuses fois, à chaque impulsion de codeur)
  if (digitalReadFast2(codeurPinA) == digitalReadFast2(codeurPinB)) {
    ticksCodeur++;
  }
  else {
    ticksCodeur--;
  }
}

void GestionInterruptionCodeurPinB() {
  // Routine de service d'interruption attachée à la voie B du codeur incrémental
  // On utilise la fonction digitalReadFast2 de la librairie digitalWriteFast
  // car les lectures doivent être très rapides pour passer le moins de temps possible
  // dans cette fonction (appelée de nombreuses fois, à chaque impulsion de codeur)
  if (digitalReadFast2(codeurPinA) == digitalReadFast2(codeurPinB)) {
    ticksCodeur--;
  }
  else {
    ticksCodeur++;
  }
}


