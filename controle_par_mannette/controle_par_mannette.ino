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
#if DEBOGGAGE
const int dt = 1000; // 1 fois par seconde en mode déboggage
#else
const int dt = 10;   // À 100 Hz en fonctionnement normal
#endif

// Zone morte du contrôle de direction
const float direction_assez_precis = 0.1;
const int direction_vitesse_moteur = 255;

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
  //vitesse_moteur = max(vitesse_f, 0.1) * 255.0;
  vitesse_moteur = 0.1 * 255.0;

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
  int direction_mesure_i = analogRead(mesure_direction);
  // Convertir la lecture analogique entière entre -1 et 1
  float direction_mesure_f = 2.0 * (direction_mesure_i - 44.0) / (1023.0 - 44.0) - 1.0;

#if DEBOGGAGE
  Serial.print(F("Direction lue au potentiometre: "));
  Serial.print(direction_mesure_i);
  Serial.print(F(", une fois convertie entre -1 et 1: ")); 
  Serial.print(direction_mesure_f);
  Serial.println(F(".")); 
#endif

  // Lire la direction voulue (utilisant le contrôle de vitesse pour l'instant)
  int direction_voulue_i = vitesse_i;
  // Convertir la lecture analogique entière entre -1 et 1
  float direction_voulue_f = 2.0 * direction_voulue_i / 1023.0 - 1.0;

  // Calculer l'erreur et la correction
  float erreur_direction = direction_voulue_f - direction_mesure_f;

  if(erreur_direction < -direction_assez_precis) {
    // Tourner les roues vers la gauche
    analogWrite(moteur_gauche, direction_vitesse_moteur);    
    analogWrite(moteur_droite, 0);    
  }
  else if(erreur_direction > direction_assez_precis) {
    // Tourner les roues vers la droite
    analogWrite(moteur_gauche, 0);
    analogWrite(moteur_droite, direction_vitesse_moteur);    
  }
  else {
    // Arrêter les roues
    analogWrite(moteur_gauche, 0);
    analogWrite(moteur_droite, 0);    
  }
  
#if DEBOGGAGE
  Serial.print(F("Direction desiree: "));
  Serial.print(direction_voulue_f);
  Serial.print(F(", erreur: ")); 
  Serial.print(erreur_direction);
  Serial.println(F(".")); 
#endif

  // Attendre un peu avant de recommencer
  delay(dt);
}
