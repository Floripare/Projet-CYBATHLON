int axeX = A0; // signal de l'axe X sur entrée A0
int axeY = A1; // signal de l'axe Y sur entrée A1
int BP8 = 8; // Bouton-poussoir en broche 7
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
float X, Y;
int Bouton;
X = analogRead (axeX);
Y = analogRead (axeY);
Bouton = digitalRead (BP8);
Serial.print ("Axe X:");
Serial.print (X, 4);
Serial.print ("V, ");
Serial.print ("Axe Y:");
Serial.print (Y, 4);
Serial.print ("V, ");
Serial.print ("Bouton:");
if (Bouton==1)
{
Serial.println (" Aucune pression sur le bouton poussoir ");
}
else
{
Serial.println (" Bouton-poussoir actif ");
}
delay (300);
}
