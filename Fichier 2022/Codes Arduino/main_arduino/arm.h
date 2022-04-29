#ifndef ARM_H
#define ARM_H


class Arm {

    protected:

        float x, y, z;
        // Coordonnées cibles comprises en cm, du poignet. 
        // Note : y est en radians.
        float x_p, y_p, z_p;
        // Coordonnées au tour précédent en cm ou radians. 

        // A in instant donné, la position réelle du bras sera comprise entre 

        float alpha, beta, eta;
        // Angles cibles pour les moteurs. Alpha correspond à l'angle du bras avec l'horizontale, beta est l'angle de l'avant bras avec l'horizontale, 
        // gamma est la somme de beta et alpha soit l'angle de l'avant bras avec le bras, et eta est l'angle du poignet avec l'avant bras.
        // L'angle du poignet avec l'horizontale est obtenu en sommant alpha, beta et gamma.
        // Delta est l'angle de rotation latérale de l'épaule.
        // Gamma est potentiellement inutile.

        float alpha_p, beta_p, eta_p;
        // Stockage pour les valeurs précédentes des angles. 

        int wrist_base_angle;

        float cpt_alpha, cpt_beta;
        // Valeurs de stockage pour les valeurs données par les capteurs. Ces valeurs seront ensuites comparées au valeurs voulues pendant l'asservissement.

        float arm, forearm, wrist;
        // Longueur des membres, en cm. 

        float input_x, input_y, input_z, is_clicked;
        // Inputs reçus, qui seront modifiés dans la fonction de traitement du serial.

        bool is_reseting;
        // cet int vaut 0 par défaut mais vaut 1 lorsque la machine est en cours de reset. Tant que cet entier vaut 1, la machine ne peut être interrompue.


    public:

        Arm(float arm, float forearm, float wrist);
        // Constructeur. A utiliser une fois au démarrage du robot pour initialiser les valeurs. 
        // Les seules valeurs à fixer avec le constructeur sont les valeurs des longueurs.


        void reset();
        // Méthode pour réinitialiser le robot

        float serial_processing(char* str);
        // Analyse le serial pour en tirer les valeurs nécessaires. Traite ces valeurs pour qu'elles soient exploitables => les valeurs des inputs sont traduites en déplacement voulu.
        // A appeler avant input_processing. 
        // Modifie donc les valeurs du bras

        void captor_update(float alpha_change, float beta_change);
        // Reçoit les valeurs des capteurs et met à jour toutes les données nécessaires dans la classe :  les angles des capteurs, mais aussi la position du robot en fonction des capteurs.
        // A lancer avant l'input processing pour éviter un décalage entre le calcul et la réalité, mais également à chaque tour de boucle pour l'asservissement.

        void input_processing();
        // Méthode réalisant le traitement des inputs et mettant à jour la position actuelle du robot enregistrée dans la classe, ainsi que les angles nécessaires. 

        void angle_processing();
        // Calcule les valeurs exactes des angles actuels du robot pour une position donnée et les enregistre.
        // Enregistre également les anciennes valeurs (pour le moment dans les portions dédiées de la classe.) 
        // Cette fonction sert lors de la réception des inputs du serial.

        // Renvoie la différence entre alpha et alpha_p
        float get_alpha_diff();

        // Renvoie la différence entre beta et beta_p
        float get_beta_diff();

        // Renvoie la différence entre la position enegistrée par le capteur et beta_p. Sert pour le poignet.
        float get_real_beta_diff();

        int get_wrist_base_angle();

};

int signof(float a);

#endif