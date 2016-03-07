/////////////////////////////////////////////////////////////
//
// norbert-base -> controle_par_manette
//
// Permet de contrôler la base Tonka de Norbert en utilisant
// une mannette fabriquée en Lego
//
// David Beaudette (2016)
//
/////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
//
// Constantes
//
/////////////////////////////////////////////////////////////

#define DEBOGGAGE 0

// Numéro de pin des sorties vers les moteurs
const int moteur_avant   =  9;
const int moteur_arriere = 10;
const int moteur_gauche  =  3;  // (à confirmer)
const int moteur_droite  = 11;  // (à confirmer)

// Numéro de pin de l'entrée de direction
const int mesure_direction = A6;

// Numéro de pin des entrées de la mannette
const int mannette_avant   =  4;
const int mannette_arriere =  7;
const int mannette_gauche  =  8; 
const int mannette_droite  = 12;  
const int mannette_vitesse = A7;  

// Durée minimale de la boucle principale
const int dt = 10;

/////////////////////////////////////////////////////////////
//
// Initialisation
//
/////////////////////////////////////////////////////////////

void setup() {
  // Initialisation de la communication série
  Serial.begin(115200);
  
  Serial.println(F("Controle de Norbert par mannette"));
  Serial.println(F("David Beaudette (2016)"));
  
  // Initialisation des sorties vers les moteurs
  pinMode(moteur_avant,   OUTPUT);  
  pinMode(moteur_arriere, OUTPUT);
  pinMode(moteur_gauche,  OUTPUT);  
  pinMode(moteur_droite,  OUTPUT); 
  
  // Initialisation des entrées de la mannette
  pinMode(mannette_avant,   INPUT_PULLUP);  
  pinMode(mannette_arriere, INPUT_PULLUP); 
  pinMode(mannette_gauche,  INPUT_PULLUP); 
  pinMode(mannette_droite,  INPUT_PULLUP); 
  
  //  Entrée de vitesse
  pinMode(mannette_vitesse,  INPUT); 
  analogReference(EXTERNAL);

  // Initialisation du PID de la direction
  //  Entrée de direction
  pinMode(mesure_direction,  INPUT); 

  
}

/////////////////////////////////////////////////////////////
//
// Programme principal
//
/////////////////////////////////////////////////////////////

void loop() {

  // Lire l'état des controles sur la manette
  int bouton_avant   = digitalRead(mannette_avant);
  int bouton_arriere = digitalRead(mannette_arriere);
  
  int vitesse_i = 1023 - analogRead(mannette_vitesse);
  int vitesse_moteur;

  // Convertir la lecture analogique entière entre 20% et 100% de 255
  float vitesse_f = (vitesse_i / 1023.0);
  vitesse_moteur = max(vitesse_f, 0.1) * 255.0;

#if DEBOGGAGE
  Serial.print(F("Vitesse au potentiometre: "));
  Serial.print(vitesse_i);
  Serial.print(F(", une fois convertie entre 0 et 1: ")); 
  Serial.print(vitesse_f);
  Serial.print(F(", entre 0 et 255: ")); 
  Serial.print(vitesse_moteur);
  Serial.println(F(".")); 
#endif

  if(bouton_avant == LOW) {
    // Activer le moteur avant
    analogWrite(moteur_avant, vitesse_moteur);
  }
  else {
    analogWrite(moteur_avant, 0);
  }
  if(bouton_arriere == LOW) {
      // Activer le moteur arrière
      analogWrite(moteur_arriere, vitesse_moteur);
  }
  else {
    analogWrite(moteur_arriere, 0);
  }
  
  // Lire la direction actuelle
  int direction_i = analogRead(mesure_direction);
  // Convertir la lecture analogique entière entre -1 et 1
  float direction_f = 2.0 * (direction_i - 44.0) / (1023.0 - 44.0) - 1.0;

#if DEBOGGAGE
  Serial.print(F("Direction au potentiometre: "));
  Serial.print(direction_i);
  Serial.print(F(", une fois convertie entre -1 et 1: ")); 
  Serial.print(direction_f);
  Serial.println(F(".")); 
#endif

  delay(dt);
}
