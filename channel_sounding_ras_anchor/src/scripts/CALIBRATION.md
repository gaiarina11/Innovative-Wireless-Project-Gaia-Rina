# Calibrazione RTLS host

Il firmware continua a produrre soltanto le distanze grezze `DIST_DATA`. Le
coordinate, la calibrazione delle distanze e il controllo di qualità sono
gestiti sul computer da `trilateration_network.py`.

Con il firmware Mode 3 ogni record contiene contemporaneamente la stima PBR e
la stima RTT. La sorgente usata viene scelta con `--distance-source phase`,
`rtt` oppure `fused`. Dataset e coefficienti ottenuti con sorgenti diverse non
devono essere mescolati; anche la vecchia calibrazione Mode 2 non è valida per
questo firmware. Per le prime prove usare `rtls_config_mode3_raw.json`, che ha
coefficienti neutri.

## 1. Impostare la geometria reale

Modificare `rtls_config.json` e sostituire `anchors_m` con le coordinate 2D
misurate dei tre centri antenna, espresse in metri. Usare Anchor 0 come origine
è comodo ma non obbligatorio. Le tre anchor non devono essere allineate.

Durante la misura mantenere tutte le antenne alla stessa quota del tag. Se le
quote sono diverse, le distanze 3D non sono direttamente compatibili con il
solver 2D.

## 2. Raccogliere punti noti

Posizionare il tag fermo in un punto misurato e avviare, per esempio:

```sh
python3 trilateration_network.py \
    --calibration-point 0.40,0.30 \
    --calibration-snapshots 100 \
    --no-plot
```

Le porte delle board attualmente assegnate sono già presenti come valori
predefiniti. È sempre possibile specificarle esplicitamente con tre opzioni
`--port ID=PORTA`.

Ripetere il comando con il tag in almeno altri due punti noti, distribuiti
nell'area di lavoro e a distanze differenti dalle anchor, per esempio:

```sh
python3 trilateration_network.py --calibration-point 1.10,0.30 --calibration-snapshots 100 --no-plot
python3 trilateration_network.py --calibration-point 0.75,0.85 --calibration-snapshots 100 --no-plot
```

Questi tre esempi presuppongono la geometria configurata A0=`(1.5,0)`,
A1=`(0,0)`, A2=`(0.75,1.2)`. Le coordinate passate al comando devono
corrispondere alla posizione fisica realmente misurata del tag: eseguire tre
comandi diversi senza spostare fisicamente il tag rende la calibrazione non
valida.

Ogni esecuzione aggiunge una sessione a `calibration_dataset.json` e aggiorna
automaticamente `rtls_config.json`:

- con un solo punto viene stimato un offset per anchor;
- con almeno due distanze sufficientemente diverse viene stimato il modello
  `d_corretta = scale * d_grezza + offset`;
- il riepilogo finale mostra modello, coefficienti, RMSE del fit e numero di
  campioni.

Prima di salvare, lo script controlla scala, offset e RMSE di tutte le anchor.
Se un fit non è plausibile stampa `ERRORE CALIBRAZIONE` e lascia invariati sia
il dataset sia `rtls_config.json`. Il limite predefinito del fit è `0.25 m` e
può essere modificato, solo per prove motivate, con
`--max-calibration-rmse`.

Non spostare il tag durante una raccolta. Se una sessione è stata acquisita con
coordinate errate, usare un nuovo percorso con `--calibration-dataset` oppure
rimuovere manualmente quella sessione dal JSON prima di rifare il fit.

## 3. Avviare la localizzazione

Terminata la calibrazione:

```sh
python3 trilateration_network.py --no-plot
```

Togliere `--no-plot` per la visualizzazione Matplotlib. Ogni posizione valida
riporta le tre distanze corrette (`Dc0`, `Dc1`, `Dc2`), l'RMSE geometrico e lo
skew temporale. Una terna con RMSE superiore a `quality.max_rmse_m` viene
scartata. La soglia può essere provata senza modificare il file tramite
`--max-rmse 0.25`.

Per una caratterizzazione Mode 3 iniziale, senza applicare la vecchia
calibrazione, registrare separatamente le tre selezioni:

```sh
python3 trilateration_network.py --config rtls_config_mode3_raw.json --distance-source phase --csv-log measurements/mode3_phase_raw.csv --no-plot
python3 trilateration_network.py --config rtls_config_mode3_raw.json --distance-source rtt --csv-log measurements/mode3_rtt_raw.csv --no-plot
python3 trilateration_network.py --config rtls_config_mode3_raw.json --distance-source fused --csv-log measurements/mode3_fused_raw.csv --no-plot
```

Per calibrare una sorgente, copiare prima la configurazione neutra in un file
dedicato (per esempio `rtls_config_mode3_rtt.json`). Usare quel file con
`--config`, un dataset dedicato come
`--calibration-dataset calibration_dataset_mode3_rtt.json`, e mantenere la
stessa opzione `--distance-source` in tutti i punti della campagna. In questo
modo la calibrazione non sovrascrive il riferimento neutro.

## Test offline

I test non richiedono le board:

```sh
python3 -m unittest discover -s tests -v
```
