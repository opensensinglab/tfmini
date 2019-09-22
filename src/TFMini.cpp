/*
Arduino driver for Benewake TFMini time-of-flight distance sensor. 
by Peter Jansen (December 11/2017)
modified for EAWAG volaser (2019)
This code is open source software in the public domain.

THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR(S) BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  

The names of the contributors may not be used to endorse or promote products
derived from this software without specific prior written permission.
*/

#include "TFMini.h"

// Constructor
TFMini::TFMini()
{
  // Empty constructor
}

boolean TFMini::begin(Stream *_streamPtr)
{
  // Store reference to stream/serial object
  streamPtr = _streamPtr;

  // Clear state
  distance = -1;
  strength = -1;
  mode = -1;
  state = READY;

  // Set standard output mode
  setStandardOutputMode();

  return true;
}

// Public: The main function to measure distance.
uint16_t TFMini::getDistance()
{
  int numMeasurementAttempts = 0;
  while (takeMeasurement() != 0)
  {
    numMeasurementAttempts += 1;
    if (numMeasurementAttempts > TFMINI_MAX_MEASUREMENT_ATTEMPTS)
    {
      Serial.println("TF Mini error: too many measurement attempts");
      Serial.println("Last error:");
      if (state == ERROR_SERIAL_NOHEADER)
        Serial.println("ERROR_SERIAL_NOHEADER");
      if (state == ERROR_SERIAL_BADCHECKSUM)
        Serial.println("ERROR_SERIAL_BADCHECKSUM");
      if (state == ERROR_SERIAL_TOOMANYTRIES)
        Serial.println("ERROR_SERIAL_TOOMANYTRIES");

      state = ERROR_SERIAL_TOOMANYTRIES;
      distance = -1;
      strength = -1;
      return -1;
    }
  }

  if (state == MEASUREMENT_OK)
  {
    return distance;
  }
  else
  {
    return -1;
  }
}

// Public: Return the most recent signal strength measuremenet from the TF Mini
uint16_t TFMini::getRecentSignalStrength()
{
  return strength;
}

// Public: Return the measurement mode
uint8_t TFMini::getMode()
{
  return mode;
}

// Private: Set the TF Mini into the correct measurement mode
void TFMini::setStandardOutputMode()
{
  // Set to "standard" output mode (this is found in the debug documents)
  streamPtr->write((uint8_t)0x42);
  streamPtr->write((uint8_t)0x57);
  streamPtr->write((uint8_t)0x02);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x01);
  streamPtr->write((uint8_t)0x06);
}

// Set configuration mode
void TFMini::setConfigMode()
{
  // advanced parameter configuration mode
  streamPtr->write((uint8_t)0x42);
  streamPtr->write((uint8_t)0x57);
  streamPtr->write((uint8_t)0x02);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x01);
  streamPtr->write((uint8_t)0x02);
  delay(100);
}

// Exit configuration mode
void TFMini::unsetConfigMode()
{
  streamPtr->write((uint8_t)0x42);
  streamPtr->write((uint8_t)0x57);
  streamPtr->write((uint8_t)0x02);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x02);
  delay(100);
}

// Set single scan mode (external trigger)
void TFMini::setSingleScanMode()
{
  setConfigMode();
  // setting trigger source to external
  streamPtr->write((uint8_t)0x42);
  streamPtr->write((uint8_t)0x57);
  streamPtr->write((uint8_t)0x02);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x40);
  delay(100);
  unsetConfigMode();
}

// Set measurement mode
void TFMini::setMeasurementMode(MODE _mode)
{
  setConfigMode();

  // fix detection pattern
  streamPtr->write((uint8_t)0x42);
  streamPtr->write((uint8_t)0x57);
  streamPtr->write((uint8_t)0x02);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x01);  //0x00=unlocked, 0x01=locked
  streamPtr->write((uint8_t)0x14);

  delay(100);

  // setting measurement mode
  streamPtr->write((uint8_t)0x42);
  streamPtr->write((uint8_t)0x57);
  streamPtr->write((uint8_t)0x02);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)_mode);
  streamPtr->write((uint8_t)0x11);

  delay(100);

  unsetConfigMode();
}

// Set maximum distance
void TFMini::setRangeLimit(uint16_t range) {
  setConfigMode();

  streamPtr->write((uint8_t)0x42);
  streamPtr->write((uint8_t)0x57);
  streamPtr->write((uint8_t)0x02);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)(range & 0xff));
  streamPtr->write((uint8_t)((range >> 8) & 0xff));
  streamPtr->write((uint8_t)(range > 0 ? 0x01 : 0x00)); // 0x00=range limit disabled, 0x01=enabled
  streamPtr->write((uint8_t)0x19);

  delay(100);

  unsetConfigMode();
}

// Set maximum distance
void TFMini::setSignalThreshold(uint8_t min, uint16_t max) {
  setConfigMode();

  // Set minimum signal strength
  streamPtr->write((uint8_t)0x42);
  streamPtr->write((uint8_t)0x57);
  streamPtr->write((uint8_t)0x02);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)min);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x20);

  delay(100);  

  // Set maximum signal strength
  streamPtr->write((uint8_t)0x42);
  streamPtr->write((uint8_t)0x57);
  streamPtr->write((uint8_t)0x02);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)(max & 0xff));
  streamPtr->write((uint8_t)((max >> 8) & 0xff));
  streamPtr->write((uint8_t)0x1d); // distance to be returned if max signal is exceeded
  streamPtr->write((uint8_t)0x21);

  delay(100); 

  unsetConfigMode();
}

// Send external trigger
void TFMini::externalTrigger()
{
  setConfigMode();
  // send trigger
  streamPtr->write((uint8_t)0x42);
  streamPtr->write((uint8_t)0x57);
  streamPtr->write((uint8_t)0x02);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x00);
  streamPtr->write((uint8_t)0x41);
  delay(100);
  unsetConfigMode();
}

// Private: Handles the low-level bits of communicating with the TFMini, and detecting some communication errors.
int TFMini::takeMeasurement()
{
  int numCharsRead = 0;
  uint8_t lastChar = 0x00;

  // Step 1: Read the serial stream until we see the beginning of the TF Mini header, or we timeout reading too many characters.
  while (1)
  {

    if (streamPtr->available())
    {
      uint8_t curChar = streamPtr->read();

      if ((lastChar == 0x59) && (curChar == 0x59))
      {
        // Break to begin frame
        break;
      }
      else
      {
        // We have not seen two 0x59's in a row -- store the current character and continue reading.
        lastChar = curChar;
        numCharsRead += 1;
      }
    }

    // Error detection:  If we read more than X characters without finding a frame header, then it's likely there is an issue with
    // the Serial connection, and we should timeout and throw an error.
    if (numCharsRead > TFMINI_MAXBYTESBEFOREHEADER)
    {
      state = ERROR_SERIAL_NOHEADER;
      distance = -1;
      strength = -1;
      mode = -1;
      if (TFMINI_DEBUGMODE == 1)
        Serial.println("ERROR: no header");
      return -1;
    }
  }

  // Step 2: Read one frame from the TFMini
  uint8_t frame[TFMINI_FRAME_SIZE];

  uint8_t checksum = 0x59 + 0x59;
  for (int i = 0; i < TFMINI_FRAME_SIZE; i++)
  {
    // Read one character
    while (!streamPtr->available())
    {
      // wait for a character to become available
    }
    frame[i] = streamPtr->read();

    // Store running checksum
    if (i < TFMINI_FRAME_SIZE - 2)
    {
      checksum += frame[i];
    }
  }

  // Step 2A: Compare checksum
  // Last byte in the frame is an 8-bit checksum
  uint8_t checksumByte = frame[TFMINI_FRAME_SIZE - 1];
  if (checksum != checksumByte)
  {
    state = ERROR_SERIAL_BADCHECKSUM;
    distance = -1;
    strength = -1;
    if (TFMINI_DEBUGMODE == 1)
      Serial.println("ERROR: bad checksum");
    return -1;
  }

  // Step 3: Interpret frame
  uint16_t dist = (frame[1] << 8) + frame[0];
  uint16_t st = (frame[3] << 8) + frame[2];
  uint8_t reserved = frame[4];
  uint8_t originalSignalQuality = frame[5];

  // Step 4: Store values
  distance = dist;
  strength = st;
  mode = reserved;
  state = MEASUREMENT_OK;

  // Return success
  return 0;
}
