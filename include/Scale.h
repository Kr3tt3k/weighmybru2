#ifndef SCALE_H
#define SCALE_H

#include <HX711.h>
#include <Preferences.h>

class Scale {
public:
    // Konstruktor für einen HX711 (abwärtskompatibel)
    Scale(uint8_t dataPin, uint8_t clockPin, float calibrationFactor);
    
    // Konstruktor für zwei HX711 mit gemeinsamen CLK-Pin
    Scale(uint8_t dataPin1, uint8_t dataPin2, uint8_t clockPin, float calibrationFactor);
    
    bool begin();  // Returns true if successful, false if HX711 fails
    void tare(uint8_t times = 20);
    void set_scale(float factor);
    float getWeight();
    float getCurrentWeight();
    long getRawValue();
    void saveCalibration(); // Save calibration factor to NVS
    void loadCalibration(); // Load calibration factor from NVS
    float getCalibrationFactor() const { return calibrationFactor; } // Getter for API
    
    // NEUE METHODEN FÜR DUAL-KALIBRIERUNG
    void setCalibrationFactors(float factor1, float factor2);
    float getCalibrationFactor1() const { return calibrationFactor1; }
    float getCalibrationFactor2() const { return calibrationFactor2; }
    long getRawValue1();
    long getRawValue2();
    void saveDualCalibration();
    void loadDualCalibration();
    
    bool isHX711Connected() const { return isConnected; } // Check if HX711 is responding
    
    // Filtering configuration - adjustable for different load cells
    void setBrewingThreshold(float threshold);
    void setStabilityTimeout(unsigned long timeout);
    void setMedianSamples(int samples);
    void setAverageSamples(int samples);
    
    float getBrewingThreshold() const { return brewingThreshold; }
    unsigned long getStabilityTimeout() const { return stabilityTimeout; }
    int getMedianSamples() const { return medianSamples; }
    int getAverageSamples() const { return averageSamples; }
    String getFilterState() const; // Get current filter state as string for debugging
    
    void saveFilterSettings();
    void loadFilterSettings();
    
    // FlowRate integration for tare operations
    void setFlowRatePtr(class FlowRate* flowRatePtr);

    // Dual HX711 status methods
    bool isDualHX711() const { return dualHX711; }
    String getHX711Status() const; // Get HX711 connection status as string
    
private:
    HX711 hx7111;           // Erster HX711
    HX711 hx7112;           // Zweiter HX711 (optional)
    Preferences preferences;
    uint8_t dataPin1;       // Erster Daten-Pin
    uint8_t dataPin2;       // Zweiter Daten-Pin (0 wenn nicht verwendet)
    uint8_t clockPin;       // Gemeinsamer CLK-Pin
    
    // NEUE KALIBRIERUNGSFAKTOREN FÜR DUAL-MODUS
    float calibrationFactor = 0.0f;      // Kombinierter Faktor (abwärtskompatibel)
    float calibrationFactor1 = 0.0f;     // Faktor für HX711 #1
    float calibrationFactor2 = 0.0f;     // Faktor für HX711 #2
    
    float currentWeight;
    bool isConnected = false;  // Track HX711 connection status
    bool dualHX711 = false;    // Zwei HX711 Module aktiv
    class FlowRate* flowRatePtr = nullptr; // For pausing flow rate during tare
    
    // Smart filtering variables - reduced buffer for faster response
    static const int MAX_SAMPLES = 10;  // Reduced from 50 to 10 for faster response
    float readings[MAX_SAMPLES];
    int readingIndex = 0;
    bool samplesInitialized = false;
    float previousFilteredWeight = 0;
    
    // Status-Tracking für stabile Erkennung
    mutable unsigned long lastSuccessfulRead = 0; // Zeitpunkt des letzten erfolgreichen Reads
    
    // Brewing state tracking for smart filtering
    enum FilterState {
        STABLE,     // Using average filter - stable weight
        BREWING,    // Using median filter - active brewing
        TRANSITIONING // Waiting for stability after brewing activity
    };
    FilterState currentFilterState = STABLE;
    unsigned long lastBrewingActivity = 0;  // Track when brewing was last detected
    float lastStableWeight = 0.0f;          // Last weight when in stable state
    
    // Configurable filtering parameters
    float brewingThreshold = 0.15f;  // Keep for API compatibility
    unsigned long stabilityTimeout = 2000;  // Keep for API compatibility
    int medianSamples = 3;  // Keep for API compatibility
    int averageSamples = 2;  // Samples for average filter - reduced for faster response
    
    // Private methods
    bool initializeSingleHX711();
    bool initializeDualHX711();
    float readSingleHX711();
    float readDualHX711();
    
    // Filter methods
    float medianFilter(int samples);
    float averageFilter(int samples);
    void initializeSamples(float initialValue);
};

#endif
