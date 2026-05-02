/**
 * File: SettingsManager.h
 * Version: 1.27
 * Description: Управление NVS и сохранение настроек/слепков (UC-03).
 */

 #ifndef SETTINGS_MANAGER_H
 #define SETTINGS_MANAGER_H
 
 #include <Arduino.h>
 #include <Preferences.h>
 #include "configuration.h"
 
 // Форвард-декларация, чтобы не было циклического включения
 class NodeDatabase; 
 
 #pragma pack(push, 1)
 struct SavedNodeRecord {
     uint8_t id;
     uint8_t role;
     char name[12];
     uint32_t packedCoords;
 };
 #pragma pack(pop)
 
 class SettingsManager {
 public:
     SettingsManager();
     
     void init();
     void save();
     void factoryReset();
 
     // Запись и чтение слепка базы (Roster + Coords)
     void saveNodesSnapshot(const NodeDatabase& db);
     void loadNodesSnapshot(NodeDatabase& db);
 
     NavigaSettings settings;
 
 private:
     Preferences _preferences;
     const char* PREFS_NAMESPACE = "naviga";
     const char* PREFS_CFG_KEY = "sys_config";
     const char* PREFS_NODES_KEY = "nodes_snap";
 
     void loadDefaults();
 };
 
 extern SettingsManager settingsManager;
 
 #endif // SETTINGS_MANAGER_H