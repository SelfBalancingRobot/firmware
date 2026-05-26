# Robot Samobalansujący
## protokół komunikacyjny
### opis
Protokół na ten moment umożliwia przesyłanie nastaw regulatorów oraz zmiane trybu pracy robota. Wszelkie komendy powinno się wysyłać pojedynczo. Każda kolejna komenda powinna być poprzedzona krótkim co najmniej kilkudziesięcio milisekundowym opóźnieniem względem poprzedniej.
### ogólna struktura wiadomości
Każda wiadomość powinna składać się z:
- identyfikatora komendy
- argumentów komendy
- znaków końca linii `'\n'`

Ogólny format:

```text
    [KOMENDA][ARGUMENTY]\n
```
### zmiana nastaw regulatorów
#### format komendy

```text
    pid[TARGET][PARAMETR][WARTOŚĆ]\n
```

| Pole | Opis |
|---|---|
| pid | typ wysyłanej komendy, w przypdaku zmiany nastaw zawsze jest to "pid" |
| TARGET | regulator w którym chcemy zmienić nastawy |
| PARAMETR | parametr regulatora, który chcemy zmienić | 
| WARTOŚĆ | wartość na jaką chcemy zmienić dany parametr, ilość cyfr przed i po '.' dowolna |

#### przykładowa komenda 
```text
    pidmp10.5\n
```
- `pid`  -> zmiana nastaw regulatora
- `m`    -> regulatory silników
- `p`    -> człon proporcjonalny (Kp)
- `10.5` -> wartość

#### Uwagi
Na ten moment nie uwzględniono możliwości zapisywania różnych nastaw do silników tzn. wysyłając komende zmiany nastaw regulatora silnika zmieniamy nastawy dla obu silników na identyczne.
### zmiana trybu pracy robota
#### format komendy

```text
   mode[TRYB_PRACY]\n
```
| Pole | Opis |
|---|---|
| mode | typ wysyłanej komendy, w przypdaku zmiany trybu pracy zawsze jest to "mode" |
| TRYB_PRACY | tryb pracy w jaki ma przejść robot |

#### przykładowa komenda
```text
    modec\n
```
- `mode` -> zmiana trybu pracy
- `c`    -> tryb kalibracji silników