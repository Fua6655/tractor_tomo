# TOMO ControlFactory – FSM Overview

Centralni **event-based state machine** za upravljanje vozilom (ESP output),
sa podrškom za **PS4 / Web / Auto** izvore i **Soft / Hard Emergency** režime.

---

## 1. Arhitektura (High level)

[ PS4 Node ] ─┐
[ Web Node ] ─┼─> ControlEvents ──┐
[ Auto Node ] ─┘ │
▼
┌─────────────────┐
│ ControlFactory │
│ (FSM + Reducer)│
└─────────────────┘
│
▼
[ OutputStates ]
│
▼
ESP



- **PS4 / Web / Auto** šalju *čiste evente*
- **ControlFactory** je *single source of truth*
- ESP nikad ne donosi logiku

---

## 2. FSM Stanja

### 2.1 ARM STATE

| State | Opis |
|------|------|
| DISARMED | Sve blokirano, engine_stop = 1 |
| ARMED | Sustav aktivan |

**Pravila:**
- Ulazak u `ARMED` → `front_position = ON`
- Izlazak iz `ARMED` → hard reset svega

---

### 2.2 POWER STATE

| State | Opis |
|------|------|
| OFF | Pogon isključen |
| ON | Pogon uključen |

**Pravila:**
- Ulazak u `POWER ON`:
  - `clutch_active = 1`
  - `brake_active = 1`
- Izlazak iz `POWER`:
  - `clutch_active = 0`
  - `brake_active = 0`
- `ENGINE_START` dozvoljen samo kad je `POWER ON`

---

### 2.3 LIGHT STATE (Permission mode)

| State | Opis |
|------|------|
| OFF | Svjetla se ne mogu mijenjati |
| ON | Dozvoljena promjena svjetala |

**VAŽNO:**  
Light state **NE predstavlja stanje svjetala**, već:
> dozvolu za promjenu konfiguracije

- Izlazak iz light moda **NE resetira svjetla**
- Eventi za svjetla se ignoriraju dok je `LIGHT OFF`

---

## 3. Front Light FSM

OFF ──> SHORT ──> LONG ──> OFF


- Aktivno samo kad je `LIGHT ON`
- Implementirano preko `front_mode`:
  - 0 = OFF
  - 1 = SHORT
  - 2 = LONG

---

## 4. Blinkeri

### Normal Mode
- Lijevi i desni blinker neovisni
- Blink faza: **0.5 s**
- Aktivni samo ako su uključeni

### Emergency Override
| Emergency | Blink period | Blinkeri |
|---------|--------------|----------|
| SOFT | 0.5 s | Oba |
| HARD | 0.25 s | Oba |

- Emergency **ignorira light mode**
- Blinkeri su forsirani bez obzira na user input

---

## 5. Emergency FSM

### SOFT EMERGENCY
- Blokira:
  - ENGINE_START
  - MOVE_ALLOWED
  - CLUTCH
  - BRAKE
- Sprema kompletno stanje
- Aktivira hazard blinking (0.5 s)

### HARD EMERGENCY
- Sve resetirano
- Hazard blinking (0.25 s)
- Ignorira sve evente

### RELEASE
- Vraća stanje samo iz SOFT emergency
- HARD emergency se ne vraća automatski

---

## 6. Event Rules (Reducer)

| Kategorija | Pravilo |
|----------|--------|
| STATE | Mijenja FSM stanja |
| EVENT | Dozvoljeno samo kad nije emergency |
| LIGHT | Dozvoljeno samo kad je LIGHT ON |
| SYSTEM | Ima prioritet |

---

## 7. Design Principles

- ControlFactory = **single source of truth**
- Nodes šalju **evente**, ne logiku
- FSM > if-else
- Emergency ima **viši prioritet od svega**
- Light mode = permission, ne reset

---

## 8. Zašto ovako?

✔ Predvidivo ponašanje  
✔ Bez side-effect bugova  
✔ Jednostavno debugiranje  
✔ Spremno za:
- hazard mode
- autonomous override
- safety certification

---

## 9. Status

**FINAL FSM – production ready** 🚜🔥
