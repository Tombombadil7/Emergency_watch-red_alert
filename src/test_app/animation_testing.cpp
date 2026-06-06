#include <Arduino.h>
#include <TFT_eSPI.h>

// הגדרת המסך וה-Sprite (Canvas) כמשתנים גלובליים
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite canvas = TFT_eSprite(&tft);

// הצהרה על הפונקציה לפני שקוראים לה
void initialize_screen();

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); } // המתנה ל-Serial להיות מוכן
    Serial.println("מתחיל את האנימציה...");

    // אתחול המסך
    tft.init();
    tft.setRotation(1); // ניתן לשנות את המספר בהתאם לכיוון התצוגה הרצוי
    
    // יצירת ה-Sprite בזיכרון (מניח שהמסך הוא 240x240 לפי הקואורדינטות שלך)
    canvas.createSprite(240, 240); 
}

void loop() {
    // פונקציית loop כבר רצה בלולאה אינסופית, אז קוראים לפונקציה ישירות
    initialize_screen();
}

void initialize_screen() {
    canvas.fillSprite(TFT_BLACK); // ניקוי המסך השחור
    
    // שימוש ב-millis כדי לייצר תנועה שרצה לנצח
    // מכפילים במהירות רצויה ועושים מודולו 360 כדי להישאר בטווח המעלות
    uint32_t time_now = millis();
    int start_angle1 = (time_now / 5) % 360; 
    int end_angle1 = (start_angle1 + 90) % 360; // תוקן מ-start_angle ל-start_angle1
    
    int start_angle2 = 180 - ((time_now / 5) % 360); 
    int end_angle2 = 180 - ((start_angle1 + 90) % 360); // תוקן מ-start_angle ל-start_angle1
    
    // שרטוט הקשת האדומה עם קצוות מעוגלים (roundEnds = true)
    canvas.drawSmoothArc(120, 120, 100, 85, start_angle1, end_angle1, TFT_RED, TFT_BLACK, true);
    canvas.drawSmoothArc(120, 120, 85, 70, start_angle2, end_angle2, TFT_RED, TFT_BLACK, true);
    
    // הוספת טקסט באמצע
    canvas.setTextColor(TFT_WHITE);
    canvas.setTextDatum(MC_DATUM); // מיישר את הטקסט למרכז לפי הקואורדינטות (120,120)
    canvas.drawString("מגדיר מכשיר, כבר נתחיל...", 120, 120, 2);
    
    // זריקת הפריים המוכן למסך הפיזי
    canvas.pushSprite(0, 0); 
    delay(20);
}