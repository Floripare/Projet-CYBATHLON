/******************************************************************************
Asservissement de vitesse d'un moteur à courant continu
Ce programme est destiné à fonctionner avec l'ensemble de 
commande de moteur à courant continu, disponible à l'adresse suivante:
http://boutique.3sigma.fr/robots/23-experience-commande-de-moteur-electrique.html

Auteur: 3Sigma
Version 3.1 - 24/01/2017
*******************************************************************************/

#include "joystick.hpp"

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0)) // définition d'une fonction signe

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

// Définitions des dimensions du robot
#define lambda 20
#define mu 27

// Définitions et déclarations pour le codeur incrémental
#define codeurPoignet 22
#define codeurDirectionPoignet 23
#define codeurCoude 20
#define codeurDirectionCoude 21
#define codeurEpaule 18
#define codeurDirectionEpaule 19
#define Nmoy 10
volatile long ticksCodeurEpaule = 0;
volatile long ticksCodeurCoude = 0;
static int indiceTicksCodeur = 0;
const double encoder_PPR = 2803 / 4;
static int ticksCodeurTabEpaule[Nmoy];
static int ticksCodeurTabCoude[Nmoy];
static int codeurDeltaPosEpaule;
static int codeurDeltaPosCoude;

// Définitions et déclarations pour le moteur à courant continu.
#define directionMoteurPoignet 9
#define pwmMoteurPoignet 8
#define directionMoteurCoude  7
#define pwmMoteurCoude  6
#define directionMoteurEpaule  4
#define pwmMoteurEpaule  5
#define directionLacet 3
#define pwmLacet 2
// ATTENTION: donner la bonne valeur de la tension d'alimentation sur la ligne ci-dessous. Ici, 12V
double tensionAlim = 12.;

// Certaines parties du programme sont exécutées à cadence fixe grâce à la bibliothèque FlexiTimer2.
// Cadence d'échantillonnage en ms
#define CADENCE_MS 10
volatile double dt = CADENCE_MS/1000.;
volatile double temps = -CADENCE_MS/1000.;

// Le moteur POIGNET est asservi en vitesse grâce à un régulateur de type proportionnel (la dynamique du système est assez faible pour se limiter à ça)
// On déclare ci-dessous les variables et paramètres nécessaires à l'asservissement et au régulateur
static double anglePoignet = 0.; // angle du poignet avec l'horizontale, en rad
static double angleConsignePoignet = 0.; // angle de consigne du poignet, en rad
static double commandePoignet = 0.; // commande en tension calculée par le correcteur
static double commande_avant_satPoignet = 0.; // valeur de la commande avant la saturation (voir ci-dessous)

// Le moteur COUDE est asservi en vitesse grâce à un régulateur de type PID
// On déclare ci-dessous les variables et paramètres nécessaires à l'asservissement et au régulateur
static double VconsigneCoude = 0.; // vitesse de consigne du moteur du coude, en rad.s-1
static double omegaCoude = 0.; // vitesse de rotation du moteur

// /!\ /!\ /!\
// L'angle est ici initialisé car nous n'avons pas implémenté les capteurs de fin de course sur le robot
// Il faut donc faire démarrer le robot dans la position adéquate
// --> Utiliser le programme en boucle ouverte pour manipuler le robot à cet effet si besoin
// /!\ /!\ /!\ 

static double angleCoude = 0.; // angele du coude avec l'épaule, en rad
static double angleCoudeDenominateur = 0.;  // angle du coude avec l'épaule utilisé pour le calcul des vitesses des moteurs, en rad, voir aide à la compréhension du code
static double commandeCoude = 0.; // commande en tension calculée par le PID
static double commande_avant_satCoude = 0.; // valeur de la commande avant la saturation (voir ci-dessous)
static double yprecCoude = 0; // mesure de la vitesse au calcul précédent
static double TfCoude = 0.02; // constante de temps de filtrage de l'action dérivée du PID
static double P_x_Coude = 0.; // valeur de l'action proportionnelle
static double I_x_Coude = 0.; // valeur de l'action intégrale
static double D_x_Coude = 0.; // valeur de l'action dérivée
// Variables intermédiaires COUDE
static double Ti_Coude = 0;
static double Td_Coude = 0;
static double Tt_Coude = 0;
static double ad_Coude = 0;
static double bd_Coude = 0;
static double br_Coude = 0;

// Le moteur EPAULE est asservi en vitesse grâce à un régulateur de type PID
// On déclare ci-dessous les variables et paramètres nécessaires à l'asservissement et au régulateur
static double VconsigneEpaule = 0.; // vitesse de consigne du moteur du epaule, en rad.s-1
static double omegaEpaule = 0.; // vitesse de rotation du moteur

// /!\ /!\ /!\
// L'angle est ici initialisé car nous n'avons pas implémenté les capteurs de fin de course sur le robot
// Il faut donc faire démarrer le robot dans la position adéquate
// --> Utiliser le programme en boucle ouverte pour manipuler le robot à cet effet si besoin
// /!\ /!\ /!\ 

static double angleEpaule = 0.;  // angle de l'épaule avec le sol, en rad
static double commandeEpaule = 0.; // commande en tension calculée par le PID
static double commande_avant_satEpaule = 0.; // valeur de la commande avant la saturation (voir ci-dessous)
static double yprecEpaule = 0; // mesure de la vitesse au calcul précédent
static double TfEpaule = 0.02; // constante de temps de filtrage de l'action dérivée du PID
static double P_x_Epaule = 0.; // valeur de l'action proportionnelle
static double I_x_Epaule = 0.; // valeur de l'action intégrale
static double D_x_Epaule = 0.; // valeur de l'action dérivée
// Variables intermédiaires EPAULE
static double Ti_Epaule = 0;
static double Td_Epaule = 0;
static double Tt_Epaule = 0;
static double ad_Epaule = 0;
static double bd_Epaule = 0;
static double br_Epaule = 0;

// Consigne de vitesse de la main
static double vMain = 0.; // vitesse de consigne de la main

// Consignes de tension du moteur de la base (pour le mouvement de lacet)
static double commandeLacet = 0.; // commande en tension de la base (boucle ouverte)
static double commande_avant_satLacet = 0.; // valeur de la commande avant la saturation

// Tensions maximales aux bornes des moteurs
static double umax = 9.; // valeur max de la tension de commande du moteur
static double umin = -9.; // valeur min (ou max en négatif) de la tension de commande du moteur

// Variable utilisée pour le correcteur du POIGNET
static double KpPoignet = 0.25; // gain proportionnel du poignet

// Variables utilisées pour les données reçues COUDE
static double KpCoude = 0.25; // gain proportionnel du PID
static double KiCoude = 1.5; // gain intégral du PID
static double KdCoude = 0.0; // gain dérivé du PID

// Variables utilisées pour les données reçues EPAULE
static double KpEpaule = 0.25; // gain proportionnel du PID
static double KiEpaule = 1.5; // gain intégral du PID
static double KdEpaule = 0.0; // gain dérivé du PID

// Constantes liées au joystick
int axeX = A0; // signal de l'axe X sur entrée A0
int axeY = A1; // signal de l'axe Y sur entrée A1
int BP8 = 8; // Bouton-poussoir en broche 7

// Variables utilisées pour stocker les amplitudes d'inclinaison du joystick
static double amplitudeVerticale = 0.;
static double amplitudeHorizontale = 0.;

//Déclaration du joystick
Joystick *joystick = new Joystick();

// Initialisations
void setup(void) {
  
  // Codeur incrémental
  pinMode(codeurDirectionCoude, INPUT);      // entrée digitale pin du codeur COUDE
  digitalWrite(codeurDirectionCoude, HIGH);  // activation de la résistance de pullup du COUDE
  pinMode(codeurDirectionEpaule, INPUT);      // entrée digitale pin du codeur EPAULE
  digitalWrite(codeurDirectionEpaule, HIGH);  // activation de la résistance de pullup EPAULE
  pinMode(codeurDirectionPoignet, INPUT);      // entrée digitale pin du codeur POIGNET
  digitalWrite(codeurDirectionPoignet, HIGH);  // activation de la résistance de pullup POIGNET
  // A chaque changement de niveau de tension sur le pin du codeur,
  // on exécute la fonction GestionInterruptionCodeurPin (définie à la fin du programme)
  attachInterrupt(digitalPinToInterrupt(codeurCoude), GestionInterruptionCodeurPinCoude, RISING);
  attachInterrupt(digitalPinToInterrupt(codeurEpaule), GestionInterruptionCodeurPinEpaule, RISING);
  attachInterrupt(digitalPinToInterrupt(codeurPoignet), GestionInterruptionCodeurPinPoignet, RISING);
  
  // Moteur à courant continu
  pinMode(directionMoteurPoignet, OUTPUT);
  pinMode(pwmMoteurPoignet, OUTPUT);
  pinMode(directionMoteurCoude, OUTPUT);
  pinMode(pwmMoteurCoude, OUTPUT);
  pinMode(directionMoteurEpaule, OUTPUT);
  pinMode(pwmMoteurEpaule, OUTPUT);
  pinMode(directionLacet, OUTPUT);
  pinMode(pwmLacet, OUTPUT);
  

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

   // Réception des données du joystick
   amplitudeVerticale = analogRead(joystick->getY()) / 1023.; // amplitude haut/bas du joystick, utilisée pour les mouvements haut/bas et avant/arriere de la main, ramenée entre -1 et 1
   amplitudeHorizontale = analogRead(joystick->getX()) / 1023.; // amplitude gauche/droite du joystick, utilisée pour le mouvement de lacet du robot, ramenée entre -1 et 1

}


// Fonction excutée sur interruption
void isrt(){

  // On calcule une moyenne glissante de la vitesse sur les Nmoy derniers échantillons
  // Nombre de ticks codeur depuis la dernière fois accumulé pour les 1à derniers échantillons
  // Ce nombre est mis à jour par les fonctions GestionInterruptionCodeurPinA et GestionInterruptionCodeurPinB,
  // exécutées à chaque interruption due à une impulsion sur la voie A ou B du codeur incrémental
  for (int i=0; i<Nmoy; i++) {
    ticksCodeurTabCoude[i] += ticksCodeurCoude;
    ticksCodeurTabEpaule[i] += ticksCodeurEpaule;
  }  
  // Une fois lu, ce nombre est remis à 0 pour pouvoir l'incrémenter de nouveau sans risquer de débordement de variable
  ticksCodeurCoude = 0;
  ticksCodeurEpaule = 0;
  
  
  // Pour l'échantillon courant, calcul de l'angle de rotation du moteur pendant la période d'échantillonnage
  codeurDeltaPosCoude = ticksCodeurTabCoude[indiceTicksCodeur];
  codeurDeltaPosEpaule = ticksCodeurTabEpaule[indiceTicksCodeur];
  // Remise à zéro du compteur d'impulsion codeur pour l'échantillon courant
  ticksCodeurTabCoude[indiceTicksCodeur] = 0;
  ticksCodeurTabEpaule[indiceTicksCodeur] = 0;
  
  // Mise à jour de l'indice d'échantillon courant
  indiceTicksCodeur++;
  if (indiceTicksCodeur==Nmoy) {
    indiceTicksCodeur = 0;
  }

  // Calcul de la vitesse de rotation des moteurs. C'est le nombre d'impulsions converti en radian, divisé par la période d'échantillonnage
  // On fait un calcul glissant sur Nmoy échantillons, d'où le Nmoy*dt
  omegaCoude = ((2.*3.141592*((double)codeurDeltaPosCoude))/encoder_PPR)/(Nmoy*dt);  // en rad/s
  omegaEpaule = ((2.*3.141592*((double)codeurDeltaPosEpaule))/encoder_PPR)/(Nmoy*dt);  // en rad/s
  
  // Calcul de la consigne en fonction des données reçues sur le joystick
  // La vitesse maximale de la main est arbitrairement fixée à 4m/s, à modifier si les moteurs ne tiennent pas
  // Une marge autour de la position nulle du joystick est laissée pour ne pas laisser des petites tensions parasites perturber une position statique du robot
  vMain = (amplitudeVerticale-0.5 > 0.1 || amplitudeVerticale-0.5 < -0.1) ? (amplitudeVerticale-0.5) * 8. : 0;

  // Calcul de la vitesse de consigne des moteurs suivant la loi de vitesse : voir annexe sur l'aide à la compréhension du code pour détail du calcul
  // A cause du problème d'inversion de la loi de vitesse près de la position tendue du coude (angleEpaule ~ 0), on limite interdit les valeurs entre -5° et +5° pour angleEpaule pour les calculs au dénominateur
  angleCoudeDenominateur = (abs(angleCoude) < 10*PI/360) ? sgn(angleCoude) * 10*PI/360 : angleCoude;
  if (joystick->getAxisChoice() == joystick->GAUCHEDROITE)
    {
      VconsigneEpaule = vMain * cos(angleEpaule + angleCoude) / (lambda * sin(angleCoudeDenominateur));
      VconsigneCoude = - vMain * (lambda * cos(angleEpaule) + mu * cos(angleEpaule + angleCoude)) / (lambda * mu * sin(angleCoudeDenominateur));
    }
    else
    {
      VconsigneEpaule = vMain * sin(angleEpaule + angleCoude) / (lambda * sin(angleCoudeDenominateur));
      VconsigneCoude = - vMain * (lambda * sin(angleEpaule) + mu * sin(angleEpaule + angleCoude)) / (lambda * mu * sin(angleCoudeDenominateur));
    }

  // Appel à la fonction qui gère l'appui sur le bouton du joystick, voir définition de le fonction dans le joystick.cpp 
  joystick->processAxisChoice();







                     
  /******* Calcul du PID pour le coude*******/

// /!\ /!\ /!\
// On n'utilise ici que l'aspect "proportionnel" du correcteur
// Les aspects "intégraux" et "dérivés" sont laissés de côté mais les variables de calcul présentes pour s'en servir sont quand même présentes si besoin
// Les constantes KpCoude, KiCoude et KdCoude sont à déterminer, elles ne sont pas encore calibrées pour une dynamique correcte du robot
// /!\ /!\ /!\ 
  
  // Paramètres intermédiaires
  Ti_Coude = KiCoude/(KpCoude+0.01);
  if (KdCoude>0) { // Si PID
    ad_Coude = TfCoude/(TfCoude+dt);
    bd_Coude = KdCoude/(TfCoude+dt);
    Td_Coude = KpCoude/KdCoude;
    Tt_Coude = sqrt(Ti_Coude*Td_Coude);
  }
  else { // Si PI
    Tt_Coude = 0.5*Ti_Coude;
  }
  br_Coude = dt/(Tt_Coude+0.01);
  
  // Terme proportionnel
  P_x_Coude = KpCoude * (VconsigneCoude - omegaCoude);
  
  // Terme dérivé
  D_x_Coude = ad_Coude * D_x_Coude - bd_Coude * (omegaCoude - yprecCoude);
  
  // Calcul de la commande avant saturation

// /!\ /!\ /!\
// Le seul terme proportionnel est ici utilisé avec P_x_Coude, rajouter D_x_Coude et I_x_Coude pour les autres termes
// /!\ /!\ /!\ 
  commande_avant_satCoude = P_x_Coude;
  
  // Application de la saturation sur la commande
  if (commande_avant_satCoude > umax) {
    commandeCoude = umax;
  }
  else if (commande_avant_satCoude < umin) {
    commandeCoude = umin;
  }
  else {
    commandeCoude = commande_avant_satCoude;
  }
  
  // Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
  I_x_Coude = I_x_Coude + KiCoude * dt * (VconsigneCoude - omegaCoude) + br_Coude * (commandeCoude - commande_avant_satCoude);
  
  // Stockage de la mesure courant pour utilisation lors du pas d'échantillonnage suivant
  yprecCoude = omegaCoude;

  /******* Fin Calcul du PID pour le coude *******/
  


  


    /******* Calcul du PID pour l'épaule*******/

// /!\ /!\ /!\
// On n'utilise ici que l'aspect "proportionnel" du correcteur
// Les aspects "intégraux" et "dérivés" sont laissés de côté mais les variables de calcul présentes pour s'en servir sont quand même présentes si besoin
// Les constantes KpEpaule, KiEpaule et KdEpaule sont à déterminer, elles ne sont pas encore calibrées pour une dynamique correcte du robot
// /!\ /!\ /!\ 
  
  // Paramètres intermédiaires
  Ti_Epaule = KiEpaule/(KpEpaule+0.01);
  if (KdEpaule>0) { // Si PID
    ad_Epaule = TfEpaule/(TfEpaule+dt);
    bd_Epaule = KdEpaule/(TfEpaule+dt);
    Td_Epaule = KpEpaule/KdEpaule;
    Tt_Epaule = sqrt(Ti_Epaule*Td_Epaule);
  }
  else { // Si PI
    Tt_Epaule = 0.5*Ti_Epaule;
  }
  br_Epaule = dt/(Tt_Epaule+0.01);
  
  // Terme proportionnel
  P_x_Epaule = KpEpaule * (VconsigneEpaule - omegaEpaule);
  
  // Terme dérivé
  D_x_Epaule = ad_Epaule * D_x_Epaule - bd_Epaule * (omegaEpaule - yprecEpaule);
  
  // Calcul de la commande avant saturation

// /!\ /!\ /!\
// Le seul terme proportionnel est ici utilisé avec P_x_Epaule, rajouter D_x_Epaule et I_x_Epaule pour les autres termes
// /!\ /!\ /!\ 

  commande_avant_satEpaule = P_x_Epaule;
  
  // Application de la saturation sur la commande
  if (commande_avant_satEpaule > umax) {
    commandeEpaule = umax;
  }
  else if (commande_avant_satEpaule < umin) {
    commandeEpaule = umin;
  }
  else {
    commandeEpaule = commande_avant_satEpaule;
  }
  
  // Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
  I_x_Epaule = I_x_Epaule + KiEpaule * dt * (VconsigneEpaule - omegaEpaule) + br_Epaule * (commandeEpaule - commande_avant_satEpaule);
  
  // Stockage de la mesure courant pour utilisation lors du pas d'échantillonnage suivant
  yprecEpaule = omegaEpaule;

  /******* Fin Calcul du PID pour l'épaule *******/





  // Calcul de l'angle de consigne du poignet
  angleConsignePoignet = - (angleEpaule + angleCoude);

  // Calcul de la commande du poignet avant saturation
  commande_avant_satPoignet = KpPoignet * (angleConsignePoignet - anglePoignet);

  // Application de la saturation sur la commande
  if (commande_avant_satPoignet > umax) {
    commandePoignet = umax;
  }
  else if (commande_avant_satPoignet < umin) {
    commandePoignet = umin;
  }
  else {
    commandePoignet = commande_avant_satPoignet;
  }

  

  
      
  // Envoi des commandes aux moteurs
  CommandeMoteur(commandeCoude, tensionAlim, directionMoteurCoude, pwmMoteurCoude);
  CommandeMoteur(commandeEpaule, tensionAlim, directionMoteurEpaule, pwmMoteurEpaule);
  CommandeMoteur(commandePoignet, tensionAlim, directionMoteurPoignet, pwmMoteurPoignet);

  // Calcul de la commande du moteur de la base
  commande_avant_satLacet = (amplitudeHorizontale-0.5 > 0.1 || amplitudeHorizontale-0.5 < -0.1) ? (amplitudeHorizontale-0.5) * 510 : 0;
  if (commande_avant_satLacet > umax) {
    commandeLacet = umax;
  }
  else if (commande_avant_satLacet < umin) {
    commandeLacet = umin;
  }
  else {
    commandeLacet = commande_avant_satLacet;
  }

  // Envoi de la commande du moteur de base
  CommandeMoteur(commandeLacet, tensionAlim, directionLacet, pwmLacet);
  

  // Incrémentation du temps courant
  temps += dt;
}


void CommandeMoteur(double commande, double tensionAlim, int directionMoteur, int pwmMoteur) {
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

void GestionInterruptionCodeurPinPoignet() {  
  // Routine de service d'interruption attachée à la voie du codeur incrémental
  // On utilise la fonction digitalReadFast2 de la librairie digitalWriteFast
  // car les lectures doivent être très rapides pour passer le moins de temps possible
  // dans cette fonction (appelée de nombreuses fois, à chaque impulsion de codeur)
  if (digitalReadFast2(codeurDirectionPoignet) == HIGH) {
    anglePoignet += 2*PI / encoder_PPR;
  }
  else {
    anglePoignet -= 2*PI / encoder_PPR;
  }
}

 
void GestionInterruptionCodeurPinCoude() {  
  // Routine de service d'interruption attachée à la voie du codeur incrémental
  // On utilise la fonction digitalReadFast2 de la librairie digitalWriteFast
  // car les lectures doivent être très rapides pour passer le moins de temps possible
  // dans cette fonction (appelée de nombreuses fois, à chaque impulsion de codeur)
  if (digitalReadFast2(codeurDirectionCoude) == HIGH) {
    ticksCodeurCoude++;
    angleCoude += 2*PI / encoder_PPR;
  }
  else {
    ticksCodeurCoude--;
    angleCoude -= 2*PI / encoder_PPR;
  }
}

void GestionInterruptionCodeurPinEpaule() {  
  // Routine de service d'interruption attachée à la voie du codeur incrémental
  // On utilise la fonction digitalReadFast2 de la librairie digitalWriteFast
  // car les lectures doivent être très rapides pour passer le moins de temps possible
  // dans cette fonction (appelée de nombreuses fois, à chaque impulsion de codeur)
  if (digitalReadFast2(codeurDirectionEpaule) == HIGH) {
    ticksCodeurEpaule++;
    angleEpaule += 2*PI / encoder_PPR;
  }
  else {
    ticksCodeurEpaule--;
    angleEpaule -= 2*PI / encoder_PPR;
  }
}
