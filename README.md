# Robot Samobalansujący
## Protokół komunikacyjny
### Opis
Protokół na ten moment umożliwia przesyłanie nastaw regulatorów oraz zmiane trybu pracy robota. Wszelkie komendy powinno się wysyłać pojedynczo. Każda kolejna komenda powinna być poprzedzona krótkim co najmniej kilkudziesięcio milisekundowym opóźnieniem względem poprzedniej.
### Ogólna struktura wiadomości
Każda wiadomość powinna składać się z:
- identyfikatora komendy
- argumentów komendy
- znaku końca linii `'\n'`

#### Ogólny format:
```text
    [KOMENDA][ARGUMENTY]\n
```
### Zmiana nastaw regulatorów
#### Format komendy
```text
    pid[TARGET][PARAMETR][WARTOŚĆ]\n
```

| Pole | Opis |
|---|---|
| pid | typ wysyłanej komendy, w przypdaku zmiany nastaw zawsze jest to `pid` |
| TARGET | regulator w którym chcemy zmienić nastawy |
| PARAMETR | parametr regulatora, który chcemy zmienić | 
| WARTOŚĆ | wartość na jaką chcemy zmienić dany parametr, ilość cyfr przed i po '.' dowolna |
#### Przykładowa komenda 
```text
    pidmp10.5\n
```
- `pid`  -> zmiana nastaw regulatora
- `m`    -> regulatory silników
- `p`    -> człon proporcjonalny (Kp)
- `10.5` -> wartość
#### Dostępne regulatory
| Identyfikator | Opis |
|---|---|
| `b` | regulator balansu |
| `m` | regulatory silników |

#### Dostępne parametry
| Identyfikator | Regulator | Opis |
|---|---|---|
| `p` | oba | wzmocnienie proporcjonalne |
| `i` | oba | wzmocnienie całkujące |
| `d` | regulator balansu | wzmocnienie różniczujące |
#### Uwagi
Na ten moment nie uwzględniono możliwości zapisywania różnych nastaw do silników tzn. wysyłając komende zmiany nastaw regulatora silnika zmieniamy nastawy dla obu silników na identyczne.
### Zmiana trybu pracy robota
#### Format komendy
```text
   mode[TRYB_PRACY]\n
```
| Pole | Opis |
|---|---|
| mode | typ wysyłanej komendy, w przypdaku zmiany trybu pracy zawsze jest to `mode` |
| TRYB_PRACY | tryb pracy w jaki ma przejść robot |
#### Przykładowa komenda
```text
    modec\n
```
- `mode` -> zmiana trybu pracy
- `c`    -> tryb kalibracji silników
#### Dostępne tryby pracy
| Identyfikator | Opis |
|---|---|
| `b` | tryb balansu, normalna praca robota |
| `c` | tryb kalibracji silników, możliwość zadawania na sztywno prędkości obrotowej silnikom |
### Rodzaj transmisji
#### Opis
Komunikacja odbywa się poprzez moduł Bluetooth HC-06.
Po sparowaniu moduł widoczny jest w systemie jako wirtualny port COM, dzięki czemu komunikacja może odbywać się jak przez standardowy interfejs UART.
#### Parametry transmisji:
- baudrate:  9600
- data bits: 8
- parity:    none
- stop bits: 1  
- encoding: ASCII