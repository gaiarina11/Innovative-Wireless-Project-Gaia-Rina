# Validazione sperimentale RTLS

La stabilità visiva della stella non dimostra da sola l'accuratezza. La modalità
di validazione confronta ogni posizione con una coordinata fisica nota e salva
sia le misure accettate sia quelle scartate.

## Prova statica in un punto noto

Misurare la posizione reale del centro antenna del tag rispetto allo stesso
sistema di riferimento usato in `rtls_config.json`. Se, per esempio, il tag è
fisicamente in `(0.65, 0.25)` metri:

```sh
python3 trilateration_network.py \
    --validation-point 0.65,0.25 \
    --validation-samples 200
```

Lo script:

1. esclude 10 posizioni iniziali di warm-up;
2. raccoglie 200 posizioni valide;
3. mostra errore medio, mediano, P95 e massimo;
4. mostra deviazione standard X/Y e disponibilità;
5. salva automaticamente un CSV e un riepilogo JSON in `measurements/`.

Il CSV conserva per ogni terna:

- sequenza, metodo e numero di campioni di ciascuna anchor;
- distanza grezza, calibrata e filtrata;
- posizione diretta del solver e posizione filtrata;
- RMSE, skew, ground truth ed errore assoluto;
- motivo dello scarto (`distance_outlier`, `rmse_rejected`, ecc.).

## Griglia consigliata

Con la geometria triangolare larga 1.5 m e alta 1.2 m usare almeno questi punti,
misurandoli realmente prima di ogni comando:

```text
(0.30, 0.20)  (0.75, 0.20)  (1.20, 0.20)
(0.45, 0.50)  (0.75, 0.50)  (1.05, 0.50)
               (0.75, 0.85)
```

Non usare una coordinata soltanto perché compare in questo elenco: il tag deve
essere collocato fisicamente nel punto corrispondente.

## Criteri indicativi per il prototipo

```text
Errore medio                 < 0.30 m
Errore P95                   < 0.50 m
Deviazione standard X/Y      < 0.15 m
Disponibilità                > 90 %
RMSE geometrico tipico       < 0.20-0.30 m
```

Riportare nel progetto anche risultati peggiori: consentono di documentare i
limiti del semplice stimatore Nordic, del multipath e della geometria ridotta.

## Logging di una prova dinamica

Per registrare un percorso senza ground truth puntuale:

```sh
python3 trilateration_network.py \
    --csv-log measurements/percorso_rettilineo.csv
```

Muovere lentamente il tag lungo una linea misurata, fermandolo alcuni secondi
all'inizio e alla fine. Il confronto tra `solver_x/y` e `filtered_x/y` nel CSV
permette di valutare il ritardo del filtro.

## Parametri dei filtri

I valori predefiniti sono:

```text
Filtro mediano distanze      5 campioni
Soglia Hampel                4 deviazioni robuste
Variazione minima ammessa    0.20 m
Alpha filtro posizione       0.35
```

Per una prova con meno latenza:

```sh
python3 trilateration_network.py \
    --distance-filter-window 3 \
    --position-alpha 0.60
```

Una finestra più grande e un alpha più piccolo rendono il grafico più stabile,
ma aumentano il ritardo e non migliorano l'accuratezza assoluta.
