# AT+O4 - Clear Obstacles Command

## Beschreibung
Der neue AT+O4 Befehl ermöglicht es, alle gespeicherten Hindernisse manuell zu löschen. Dies ist besonders nützlich, wenn der Roboter durch falsche Hinderniserkennung (z.B. durch Festfahren und Drehen auf der Stelle) "Geister-Hindernisse" erstellt hat, die eine ordnungsgemäße Pfadplanung verhindern.

## Verwendung
```
AT+O4
```

## Antwort
```
O4
```

## Funktionalität
- Löscht alle Hindernisse aus der Karte (`maps.clearObstacles()`)
- Funktioniert sowohl mit dem klassischen Hindernis-System als auch mit dem Smart Obstacle System
- Loggt das Event `EVT_USER_CLEAR_OBSTACLES` für Debugging-Zwecke
- Gibt eine Bestätigungsmeldung auf der Konsole aus

## Anwendungsfall
Wenn der Roboter:
1. Sich festgefahren hat und auf der Stelle gedreht hat
2. Dadurch falsche Hindernisse erstellt hat
3. Auch nach manueller Versetzung nicht mehr zur Ladestation fahren kann
4. Kontinuierlich im Kreis fährt

Dann kann mit `AT+O4` die Hindernis-Karte zurückgesetzt werden, sodass der Roboter wieder normal navigieren kann.

## Kompatibilität
- Vollständig rückwärtskompatibel mit bestehenden AT+ Befehlen
- Erweitert die O-Befehlsfamilie (AT+O, AT+O2, AT+O3, AT+O4)
- Funktioniert mit allen Konfigurationen (mit und ohne Smart Obstacle System)

## Sicherheit
- Der Befehl löscht nur die dynamisch erstellten Hindernisse
- Statische Kartendaten (Perimeter, Ausschlusszonen) bleiben unverändert
- Kann jederzeit sicher verwendet werden