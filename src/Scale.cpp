#include "Scale.h"
#include "WebServer.h"
#include "Calibration.h"
#include "FlowRate.h"

// Konstruktor für einen HX711 (abwärtskompatibel)
Scale::Scale(uint8_t dataPin, uint8_t clockPin, float calibrationFactor)
    : dataPin1(dataPin), dataPin2(0), clockPin(clockPin), calibrationFactor(calibrationFactor), 
      currentWeight(0.0f), readingIndex(0), samplesInitialized(false), previousFilteredWeight(0), 
      medianSamples(3), averageSamples(2), currentFilterState(STABLE), lastBrewingActivity(0), 
      lastStableWeight(0.0f), dualHX711(false), lastSuccessfulRead(0) {
    // Initialize readings array
    for (int i = 0; i < MAX_SAMPLES; i++) {
        readings[i] = 0.0f;
    }
}

// Konstruktor für zwei HX711 mit gemeinsamen CLK-Pin
Scale::Scale(uint8_t dataPin1, uint8_t dataPin2, uint8_t clockPin, float calibrationFactor)
    : dataPin1(dataPin1), dataPin2(dataPin2), clockPin(clockPin), calibrationFactor(calibrationFactor),
      currentWeight(0.0f), readingIndex(0), samplesInitialized(false), previousFilteredWeight(0),
      medianSamples(3), averageSamples(2), currentFilterState(STABLE), lastBrewingActivity(0),
      lastStableWeight(0.0f), dualHX711(true), lastSuccessfulRead(0) {
    // Initialize readings array
    for (int i = 0; i < MAX_SAMPLES; i++) {
        readings[i] = 0.0f;
    }
}

bool Scale::begin() {
    Serial.println("Starting scale initialization...");
    
    preferences.begin("scale", false);
    
    if (dualHX711) {
        calibrationFactor1 = preferences.getFloat("calib1", calibrationFactor);
        calibrationFactor2 = preferences.getFloat("calib2", calibrationFactor);
        calibrationFactor = (calibrationFactor1 + calibrationFactor2) / 2.0f;
    } else {
        calibrationFactor = preferences.getFloat("calib", calibrationFactor);
    }
    
    // Load filtering parameters with load cell-specific defaults
    loadFilterSettings();
    
    // Auto-adjust brewing threshold based on calibration factor and load cell characteristics
    if (!preferences.isKey("brew_thresh")) {
        if (calibrationFactor < 1000) {
            brewingThreshold = 0.25f;
            Serial.println("Auto-detected 3kg load cell (low calibration factor)");
        } else if (calibrationFactor < 2500) {
            brewingThreshold = 0.15f;
            Serial.println("Auto-detected medium sensitivity load cell");
        } else {
            brewingThreshold = 0.1f;
            Serial.println("Auto-detected high sensitivity load cell (500g/2mV/V type)");
        }
        saveFilterSettings();
    }
    
    preferences.end();
    
    bool initializationSuccess = false;
    
    if (dualHX711) {
        Serial.println("Initializing dual HX711 configuration...");
        Serial.println("Data Pin 1: " + String(dataPin1));
        Serial.println("Data Pin 2: " + String(dataPin2));
        Serial.println("Shared Clock Pin: " + String(clockPin));
        initializationSuccess = initializeDualHX711();
    } else {
        Serial.println("Initializing single HX711 configuration...");
        Serial.println("Data Pin: " + String(dataPin1));
        Serial.println("Clock Pin: " + String(clockPin));
        initializationSuccess = initializeSingleHX711();
    }
    
    if (initializationSuccess) {
        Serial.println("Smart Scale filtering configured:");
        Serial.println("Brewing threshold: " + String(brewingThreshold) + "g");
        Serial.println("Stability timeout: " + String(stabilityTimeout) + "ms");
        Serial.println("Median samples (brewing): " + String(medianSamples));
        Serial.println("Average samples (stable): " + String(averageSamples));
        Serial.println("Smart filtering: ENABLED - Dynamic filter switching based on brewing activity");
        
        if (dualHX711) {
            Serial.printf("Dual HX711 - Calib1: %.6f, Calib2: %.6f\n", 
                         calibrationFactor1, calibrationFactor2);
        } else {
            Serial.printf("Single HX711 - Calib: %.6f\n", calibrationFactor);
        }
        
        // Initialen successful read setzen
        lastSuccessfulRead = millis();
        
        return true;
    } else {
        Serial.println("ERROR: HX711 initialization failed!");
        isConnected = false;
        return false;
    }
}

bool Scale::initializeSingleHX711() {
    // Initialize single HX711
    hx7111.begin(dataPin1, clockPin);
    hx7111.set_scale(calibrationFactor);
    
    // Test connection
    Serial.println("Testing HX711 connection...");
    unsigned long startTime = millis();
    
    while (millis() - startTime < 3000) {
        if (hx7111.is_ready()) {
            long testReading = hx7111.read();
            if (testReading != 0) {
                Serial.println("HX711 connected successfully");
                Serial.println("Test reading: " + String(testReading));
                isConnected = true;
                
                // Perform initial tare
                Serial.println("Performing initial tare...");
                hx7111.tare();
                return true;
            }
        }
        delay(100);
    }
    
    Serial.println("ERROR: Single HX711 not responding!");
    return false;
}

bool Scale::initializeDualHX711() {
    // Initialize both HX711 modules with shared clock pin
    hx7111.begin(dataPin1, clockPin);
    hx7112.begin(dataPin2, clockPin);
    
    hx7111.set_scale(calibrationFactor1);
    hx7112.set_scale(calibrationFactor2);
    
    // Test both connections
    Serial.println("Testing dual HX711 connections...");
    unsigned long startTime = millis();
    bool hx7111Ready = false;
    bool hx7112Ready = false;
    
    while (millis() - startTime < 3000) {
        if (hx7111.is_ready() && !hx7111Ready) {
            long testReading1 = hx7111.read();
            if (testReading1 != 0) {
                hx7111Ready = true;
                Serial.println("HX711 #1 connected - Raw: " + String(testReading1));
            }
        }
        
        if (hx7112.is_ready() && !hx7112Ready) {
            long testReading2 = hx7112.read();
            if (testReading2 != 0) {
                hx7112Ready = true;
                Serial.println("HX711 #2 connected - Raw: " + String(testReading2));
            }
        }
        
        if (hx7111Ready && hx7112Ready) {
            break;
        }
        delay(100);
    }
    
    if (hx7111Ready && hx7112Ready) {
        Serial.println("Both HX711 modules connected successfully");
        isConnected = true;
        
        // Perform initial tare on both modules
        Serial.println("Performing initial tare on both HX711 modules...");
        
        // Tare sequentially to avoid conflicts on shared clock line
        hx7111.tare();
        delay(100);
        hx7112.tare();
        
        return true;
    } else {
        Serial.println("ERROR: One or both HX711 modules not responding!");
        Serial.println("HX711 #1: " + String(hx7111Ready ? "OK" : "FAILED"));
        Serial.println("HX711 #2: " + String(hx7112Ready ? "OK" : "FAILED"));
        return false;
    }
}

void Scale::tare(uint8_t times) {
    if (!isConnected) {
        Serial.println("Cannot tare: HX711 not connected");
        return;
    }
    
    // Pause flow rate calculation to prevent tare operation from affecting flow rate
    if (flowRatePtr != nullptr) {
        flowRatePtr->pauseCalculation();
    }
    
    Serial.println("Taring scale...");
    
    if (dualHX711) {
        // Tare both modules sequentially
        hx7111.tare(times);
        delay(50); // Small delay between tare operations
        hx7112.tare(times);
    } else {
        hx7111.tare(times);
    }
    
    Serial.println("Tare complete");
    
    // Reset smart filter state after taring - return to stable mode
    currentFilterState = STABLE;
    lastBrewingActivity = 0;
    currentWeight = 0.0f;
    lastStableWeight = 0.0f;
    
    // Reinitialize sample buffer
    samplesInitialized = false;
    Serial.println("Smart filter reset to STABLE state");
    
    // Resume flow rate calculation after a short delay to ensure stable readings
    if (flowRatePtr != nullptr) {
        delay(100); // Short delay to let scale stabilize
        flowRatePtr->resumeCalculation();
    }
}

void Scale::set_scale(float factor) {
    // Only save if the calibration factor actually changed
    if (calibrationFactor != factor) {
        calibrationFactor = factor;
        if (dualHX711) {
            // Im Dual-Modus beide Faktoren gleich setzen
            calibrationFactor1 = factor;
            calibrationFactor2 = factor;
            hx7111.set_scale(calibrationFactor1);
            hx7112.set_scale(calibrationFactor2);
        } else {
            hx7111.set_scale(calibrationFactor);
        }
        saveCalibration();
    }
}

// NEUE METHODEN FÜR DUAL-KALIBRIERUNG
void Scale::setCalibrationFactors(float factor1, float factor2) {
    if (dualHX711) {
        calibrationFactor1 = factor1;
        calibrationFactor2 = factor2;
        calibrationFactor = (factor1 + factor2) / 2.0f; // Kombinierter Faktor
        
        hx7111.set_scale(calibrationFactor1);
        hx7112.set_scale(calibrationFactor2);
        
        saveDualCalibration();
        
        Serial.printf("Dual calibration factors set: Cell1=%.6f, Cell2=%.6f\n", 
                     calibrationFactor1, calibrationFactor2);
    } else {
        // Single-Modus: Verwende nur einen Faktor
        set_scale(factor1);
    }
}

long Scale::getRawValue1() {
    if (!isConnected || !dualHX711) return 0;
    return hx7111.get_value(1);
}

long Scale::getRawValue2() {
    if (!isConnected || !dualHX711) return 0;
    return hx7112.get_value(1);
}

void Scale::saveDualCalibration() {
    if (!dualHX711) return;
    
    preferences.begin("scale", false);
    preferences.putFloat("calib1", calibrationFactor1);
    preferences.putFloat("calib2", calibrationFactor2);
    preferences.end();
    
    Serial.printf("Dual calibration saved: %.6f, %.6f\n", calibrationFactor1, calibrationFactor2);
}

void Scale::loadDualCalibration() {
    if (!dualHX711) return;
    
    preferences.begin("scale", true);
    calibrationFactor1 = preferences.getFloat("calib1", calibrationFactor);
    calibrationFactor2 = preferences.getFloat("calib2", calibrationFactor);
    calibrationFactor = (calibrationFactor1 + calibrationFactor2) / 2.0f;
    preferences.end();
}

void Scale::saveCalibration() {
    preferences.begin("scale", false);
    if (dualHX711) {
        preferences.putFloat("calib1", calibrationFactor1);
        preferences.putFloat("calib2", calibrationFactor2);
    } else {
        preferences.putFloat("calib", calibrationFactor);
    }
    preferences.end();
}

void Scale::loadCalibration() {
    preferences.begin("scale", true);
    calibrationFactor = preferences.getFloat("calib", calibrationFactor);
    preferences.end();
}

float Scale::getWeight() {
    // Return 0 if HX711 is not connected
    if (!isConnected) {
        return 0.0f;
    }
    
    static unsigned long lastReadTime = 0;
    unsigned long currentTime = millis();
    
    // Read at 50Hz (every 20ms) for good responsiveness
    if (currentTime - lastReadTime < 20) {
        return currentWeight;
    }
    lastReadTime = currentTime;

    float rawReading;
    
    // Read from appropriate configuration
    if (dualHX711) {
        rawReading = readDualHX711();
    } else {
        rawReading = readSingleHX711();
    }
    
    // Handle invalid readings
    if (isnan(rawReading)) {
        return currentWeight;
    }
    
    // ✅ SUCCESSFUL READ - Update timestamp for status detection
    lastSuccessfulRead = currentTime;
    
    // Initialize sample buffer on first valid reading
    if (!samplesInitialized) {
        initializeSamples(rawReading);
        currentWeight = rawReading;
        lastStableWeight = rawReading;
        currentFilterState = STABLE;
        return currentWeight;
    }
    
    // Store reading in circular buffer
    readings[readingIndex] = rawReading;
    readingIndex = (readingIndex + 1) % MAX_SAMPLES;
    
    // Smart filtering based on brewing activity detection
    float weightChange = abs(rawReading - currentWeight);
    bool brewingDetected = false;
    
    // Detect brewing activity using configurable threshold
    if (currentFilterState == STABLE) {
        // Check if weight change exceeds brewing threshold
        if (weightChange > brewingThreshold) {
            brewingDetected = true;
            currentFilterState = BREWING;
            lastBrewingActivity = currentTime;
        }
    } else if (currentFilterState == BREWING) {
        // Continue monitoring for brewing activity
        if (weightChange > brewingThreshold) {
            brewingDetected = true;
            lastBrewingActivity = currentTime;
        } else {
            // Check if we should transition to stable
            if (currentTime - lastBrewingActivity > stabilityTimeout) {
                currentFilterState = TRANSITIONING;
            }
        }
    } else if (currentFilterState == TRANSITIONING) {
        // In transition phase - verify stability
        if (weightChange > brewingThreshold) {
            // Activity detected again - back to brewing
            brewingDetected = true;
            currentFilterState = BREWING;
            lastBrewingActivity = currentTime;
        } else if (currentTime - lastBrewingActivity > stabilityTimeout * 2) {
            // Extended stability confirmed - switch to stable mode
            currentFilterState = STABLE;
            lastStableWeight = currentWeight;
        }
    }
    
    // Apply appropriate filter based on current state
    float filteredWeight;
    switch (currentFilterState) {
        case BREWING:
            // Use median filter during brewing for noise rejection
            filteredWeight = medianFilter(medianSamples);
            break;
        case STABLE:
        case TRANSITIONING:
            // Use average filter for stable readings - smoother and faster
            filteredWeight = averageFilter(averageSamples);
            break;
    }
    
    // Handle rapid changes (>5g) with immediate response regardless of filter state
    if (weightChange > 5.0f) {
        filteredWeight = rawReading;
        // Reset sample buffer for immediate response
        initializeSamples(rawReading);
        // Update state appropriately
        if (currentFilterState == STABLE) {
            currentFilterState = BREWING;
            lastBrewingActivity = currentTime;
        }
    }
    
    currentWeight = filteredWeight;
    return currentWeight;
}

float Scale::readSingleHX711() {
    if (!hx7111.is_ready()) {
        return currentWeight;  // Return last known value if not ready
    }
    return hx7111.get_units(1);
}

float Scale::readDualHX711() {
    // Read from both HX711 modules and combine the readings
    if (!hx7111.is_ready() || !hx7112.is_ready()) {
        return currentWeight;  // Return last known value if not ready
    }
    
    float reading1 = hx7111.get_units(1);
    float reading2 = hx7112.get_units(1);
    
    // KORREKTUR: Beide Zellen sind individuell kalibriert und zeigen 
    // jeweils das GESAMTGEWICHT an, das sie tragen würden
    // Daher müssen wir die Werte ADDITION um das Gesamtgewicht zu erhalten
    float combinedWeight = reading1 + reading2;
    
    // Debug output
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 5000) {
        Serial.printf("Dual HX711 - Cell1: %.2fg, Cell2: %.2fg, Total: %.2fg\n", 
                     reading1, reading2, combinedWeight);
        lastDebug = millis();
    }
    
    return combinedWeight;
}

float Scale::getCurrentWeight() {
    return currentWeight;
}

long Scale::getRawValue() {
    if (!isConnected) {
        return 0;
    }
    
    if (dualHX711) {
        // KORREKTUR: Rohwerte addieren für korrekte Kalibrierung
        long value1 = hx7111.get_value(1);
        long value2 = hx7112.get_value(1);
        return value1 + value2;
    } else {
        return hx7111.get_value(1);
    }
}

String Scale::getHX711Status() const {
    if (!isConnected) {
        return "DISCONNECTED";
    }
    
    // Einfache Heuristik: Wenn innerhalb der letzten 5 Sekunden erfolgreich gelesen wurde = OK
    bool isRecentlyActive = (millis() - lastSuccessfulRead) < 5000;
    
    if (dualHX711) {
        // Im Dual-Modus gehen wir davon aus, dass beide funktionieren, wenn wir connected sind
        // und regelmäßig Werte bekommen
        return isRecentlyActive ? "DUAL_BOTH_OK" : "DUAL_BOTH_FAILED";
    } else {
        return isRecentlyActive ? "SINGLE_OK" : "SINGLE_FAILED";
    }
}

void Scale::initializeSamples(float initialValue) {
    for (int i = 0; i < MAX_SAMPLES; i++) {
        readings[i] = initialValue;
    }
    samplesInitialized = true;
}

float Scale::medianFilter(int samples) {
    if (samples > MAX_SAMPLES) samples = MAX_SAMPLES;
    
    // Copy recent readings
    float temp[samples];
    for (int i = 0; i < samples; i++) {
        int idx = (readingIndex - 1 - i + MAX_SAMPLES) % MAX_SAMPLES;
        temp[i] = readings[idx];
    }
    
    // Simple bubble sort for median (efficient for small arrays)
    for (int i = 0; i < samples - 1; i++) {
        for (int j = 0; j < samples - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                float swap = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = swap;
            }
        }
    }
    return temp[samples / 2]; // Return median
}

float Scale::averageFilter(int samples) {
    if (samples > MAX_SAMPLES) samples = MAX_SAMPLES;
    
    float sum = 0;
    int validSamples = 0;
    
    // Calculate average of recent samples
    for (int i = 0; i < samples; i++) {
        int idx = (readingIndex - 1 - i + MAX_SAMPLES) % MAX_SAMPLES;
        sum += readings[idx];
        validSamples++;
    }
    
    return sum / validSamples;
}

// Filter parameter setters with validation
void Scale::setBrewingThreshold(float threshold) {
    if (threshold >= 0.05f && threshold <= 1.0f) {
        brewingThreshold = threshold;
        saveFilterSettings();
    }
}

void Scale::setStabilityTimeout(unsigned long timeout) {
    if (timeout >= 500 && timeout <= 10000) {
        stabilityTimeout = timeout;
        saveFilterSettings();
    }
}

void Scale::setMedianSamples(int samples) {
    if (samples >= 1 && samples <= MAX_SAMPLES) {
        medianSamples = samples;
        saveFilterSettings();
    }
}

void Scale::setAverageSamples(int samples) {
    if (samples >= 1 && samples <= MAX_SAMPLES) {
        averageSamples = samples;
        saveFilterSettings();
    }
}

void Scale::saveFilterSettings() {
    preferences.begin("scale", false);
    preferences.putFloat("brew_thresh", brewingThreshold);
    preferences.putULong("stab_timeout", stabilityTimeout);
    preferences.putInt("median_samples", medianSamples);
    preferences.putInt("avg_samples", averageSamples);
    preferences.end();
    Serial.println("Filter settings saved to EEPROM");
}

void Scale::loadFilterSettings() {
    preferences.begin("scale", true);
    // Load with sensible defaults
    brewingThreshold = preferences.getFloat("brew_thresh", 0.15f);
    stabilityTimeout = preferences.getULong("stab_timeout", 2000);
    medianSamples = preferences.getInt("median_samples", 3);
    averageSamples = preferences.getInt("avg_samples", 2);
    preferences.end();
}

void Scale::setFlowRatePtr(FlowRate* flowRatePtr) {
    this->flowRatePtr = flowRatePtr;
}

String Scale::getFilterState() const {
    switch (currentFilterState) {
        case STABLE: return "STABLE";
        case BREWING: return "BREWING";
        case TRANSITIONING: return "TRANSITIONING";
        default: return "UNKNOWN";
    }
}
