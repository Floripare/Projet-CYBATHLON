#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

class Joystick
{

public:

    enum Direction {GAUCHEDROITE = false, HAUTBAS = true};

private:

    int pinAxisX, pinAxisY, pinPushButton; // Variables des pins de réception des données envoyées par le joystick

    bool axisChoice, buttonPushed, releaseWait; // Booléens liés à la commande du choix de direction (haut/bas ou droite/gauche)

public:

    /* Constructeurs */
    Joystick();
    Joystick(int valueX, int valueY, int valueButton);
    Joystick(Joystick& anotherJoystick);


    /* Getters */
    int getX();
    int getY();
    int getButtonPin();
    bool getAxisChoice();
    bool getButtonPushed();
    bool getReleaseWait();

    /* Setters */
    void setButtonPushed();
    void invertAxisChoice();
    void invertReleaseWait();

    void processAxisChoice();

};

#endif
