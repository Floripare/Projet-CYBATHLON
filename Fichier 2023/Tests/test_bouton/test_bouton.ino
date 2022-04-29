int axeX = A0; // signal de l'axe X sur entrée A0
int axeY = A1; // signal de l'axe Y sur entrée A1
int BP8 = 8; // Bouton-poussoir en broche 7

boolean etatBouton = 0;


void setup ()
{
pinMode (axeX, INPUT); // définition de A0 comme une entrée
pinMode (axeY, INPUT); // définition de A1 comme une entrée
pinMode (BP8, INPUT); // définition de 8 comme une entrée
digitalWrite(BP8, HIGH); // Activation de la résistance de Pull-Up interne de la carte Uno

Serial.begin (9600);
}


void loop ()
{

Serial.print(etatBouton);

boolean etatPinBouton = digitalRead(BP8);

if (!etatPinBouton)
{
  if (etatBouton)
  {
    etatBouton = 0;
  }
  else
  {
  etatBouton = 1;
  }
}
delay (200);
}
