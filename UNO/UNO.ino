#include <SPI.h>
#include <MFRC522.h>

// Define the pins used for the RC522 module
#define RST_PIN   9
#define SS_PIN    10

// Create an MFRC522 instance
MFRC522 mfrc522(SS_PIN, RST_PIN);

// --- Configuration for Authorized UIDs ---
// IMPORTANT: All UIDs in this list MUST have the same length.
// If you have UIDs of different lengths, you'll need a more complex structure or separate lists.
const byte UID_LENGTH = 4; // Set this to the common length of your authorized UIDs

// Define your authorized UIDs here as static const byte arrays
// Paste the UIDs you copied from the serial monitor in the previous step.
static const byte authorizedUid1[UID_LENGTH] = {0xA3, 0xAF, 0x34, 0x14}; // Example: Replace with your first UID
static const byte authorizedUid2[UID_LENGTH] = {0x43, 0xAB, 0xCE, 0x26}; // Example: Replace with your second UID
// Add more authorized UIDs as needed:
// static const byte authorizedUid3[UID_LENGTH] = {0xDE, 0xAD, 0xBE, 0xEF};
// static const byte authorizedUid4[UID_LENGTH] = {0xAA, 0xBB, 0xCC, 0xDD};

// To make it easier to loop through them, create an array of pointers to these UIDs
static const byte* const authorizedUidList[] = {
  authorizedUid1,
  authorizedUid2
  // authorizedUid3, // Add pointers here if you define more UIDs above
  // authorizedUid4
};

const int NUM_AUTHORIZED_UIDS = sizeof(authorizedUidList) / sizeof(authorizedUidList[0]);

void setup() {
  Serial.begin(115200); // Match your serial monitor
  while (!Serial);

  SPI.begin();
  mfrc522.PCD_Init();
  delay(4);

  Serial.println("RC522 Static Authorized UID Check");
  Serial.print("Checking against ");
  Serial.print(NUM_AUTHORIZED_UIDS);
  Serial.println(" hardcoded UIDs.");
  Serial.print("Expected UID length: ");
  Serial.println(UID_LENGTH);
  Serial.println();
  Serial.println("Authorized UIDs are:");
  for(int i=0; i < NUM_AUTHORIZED_UIDS; i++){
    Serial.print("  UID #"); Serial.print(i+1); Serial.print(": ");
    printSingleUid(authorizedUidList[i], UID_LENGTH);
    Serial.println();
  }
  Serial.println("------------------------------------");
  Serial.println("Scan a tag...");
}

void loop() {
  // Look for new cards
  if (!mfrc522.PICC_IsNewCardPresent()) {
    delay(50);
    return;
  }

  // Select one of the cards and read its UID
  if (!mfrc522.PICC_ReadCardSerial()) {
    delay(50);
    return;
  }

  // At this point, a card is present, and its UID is in mfrc522.uid.uidByte[]
  // and its size in mfrc522.uid.size

  Serial.print("Scanned UID: ");
  printSingleUid(mfrc522.uid.uidByte, mfrc522.uid.size);

  // First, check if the scanned UID has the expected length
  if (mfrc522.uid.size != UID_LENGTH) {
    Serial.println(" -> ACCESS DENIED (Incorrect UID length).");
  } else {
    bool isAuthorized = false;
    // Iterate through the list of authorized UIDs
    for (int i = 0; i < NUM_AUTHORIZED_UIDS; i++) {
      if (compareUids(mfrc522.uid.uidByte, authorizedUidList[i], UID_LENGTH)) {
        isAuthorized = true;
        Serial.print(" -> MATCH with authorized UID #"); Serial.print(i+1);
        break; // Found a match, no need to check further
      }
    }

    if (isAuthorized) {
      Serial.println(" -> ACCESS GRANTED!");
      // Add action for granted access (e.g., turn on LED, open lock)
      // digitalWrite(13, HIGH); delay(500); digitalWrite(13, LOW);
    } else {
      Serial.println(" -> ACCESS DENIED (UID not in authorized list).");
    }
  }

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  delay(1000); // Delay for readability
}

// Helper function to print a single UID
void printSingleUid(const byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0x0" : " 0x"); // Print in 0xHEX format for clarity
    Serial.print(buffer[i], HEX);
  }
}

// Function to compare two UIDs of a given length
// Returns true if they are identical, false otherwise
bool compareUids(const byte *uid1, const byte *uid2, byte length) {
  for (byte i = 0; i < length; i++) {
    if (uid1[i] != uid2[i]) {
      return false; // Mismatch found
    }
  }
  return true; // All bytes matched
}