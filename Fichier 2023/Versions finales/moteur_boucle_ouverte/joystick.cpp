#include <Arduino.h>
#include "joystick.hpp"


Joystick::Joystick(): // Sur le constructeur de base, les entrées sont à brancher sur les pins suivants
    pinAxisX(A0), pinAxisY(A1), pinPushButton(8),
    axisChoice(GAUCHEDROITE), buttonPushed(true), releaseWait(false)
    {}

Joystick::Joystick(int valueX, int valueY, int valueButton):
    pinAxisX(valueX), pinAxisY(valueY), pinPushButton(valueButton),
    axisChoice(GAUCHEDROITE), buttonPushed(true), releaseWait(false)
    {}

Joystick::Joystick(Joystick& anotherJoystick):
    pinAxisX(anotherJoystick.pinAxisX), pinAxisY(anotherJoystick.pinAxisY), pinPushButton(anotherJoystick.pinPushButton),
    axisChoice(anotherJoystick.axisChoice), buttonPushed(true), releaseWait(false)
    {}
    

int Joystick::getX()
{
    return pinAxisX;
}

int Joystick::getY()
{
    return pinAxisY;
}

int Joystick::getButtonPin()
{
    return pinPushButton;
}

bool Joystick::getAxisChoice()
{
    return axisChoice;
}

bool Joystick::getButtonPushed()
{
    return buttonPushed;
}

bool Joystick::getReleaseWait()
{
    return releaseWait;
}

void Joystick::setButtonPushed()
{
    buttonPushed = digitalRead(pinPushButton);
}

void Joystick::invertAxisChoice()
{
    axisChoice = !axisChoice;
}

void Joystick::invertReleaseWait()
{
    releaseWait = !releaseWait;
}

void Joystick::processAxisChoice()
{
    Joystick::setButtonPushed();
    if (Joystick::getReleaseWait() && Joystick::getButtonPushed())
    {
        Joystick::invertAxisChoice();
        Joystick::invertReleaseWait();
    }
    else if (!Joystick::getReleaseWait() && !Joystick::getButtonPushed())
    {
        Joystick::invertReleaseWait();
    }
}
