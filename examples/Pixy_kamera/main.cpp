#include <Pixy2.h>
#include <SPI.h>

Pixy2 pixy;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
  pixy.init();
  pixy.changeProg("color_connected_components"); 
}

void loop() {
  pixy.ccc.getBlocks();  

  Serial.print("Number of blocks: ");
  Serial.println(pixy.ccc.numBlocks);  

  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks - 1; i++) {
      for (int j = i + 1; j < pixy.ccc.numBlocks; j++) {
        if (pixy.ccc.blocks[i].m_x > pixy.ccc.blocks[j].m_x) {
          // Swap bloků
          auto temp = pixy.ccc.blocks[i];
          pixy.ccc.blocks[i] = pixy.ccc.blocks[j];
          pixy.ccc.blocks[j] = temp;
        }
      }
    }

    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      Serial.print("Block ");
      Serial.print(i + 1);  
      Serial.print(" - X: ");
      Serial.print(pixy.ccc.blocks[i].m_x);  // Pozice X 
      Serial.print(" Y: ");
      Serial.print(pixy.ccc.blocks[i].m_y);  // Pozice Y 
      Serial.print(" Sign: ");
      

      if (pixy.ccc.blocks[i].m_signature == 1) {
        Serial.print("RED"); 
      } else if (pixy.ccc.blocks[i].m_signature == 2) {
        Serial.print("BLUE"); 
      } else {
        Serial.print("Other");  
      }

      Serial.println();  
    }
  } else {
    Serial.println("No blocks detected.");  // Pokud Pixy2 nevidí žádné bloky
  }

  delay(1000);  // Pauza před dalším zpracováním
}