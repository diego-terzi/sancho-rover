# models/

Questa cartella contiene il modello ONNX per la segmentazione istanza del nastro blu.

## File atteso

| File | Descrizione |
|---|---|
| `trail_segmentation.onnx` | Modello RF-DETR Segmentation esportato da Roboflow |

## Come ottenere il modello (TODO Giacomo)

1. Completa il training su Roboflow (progetto Instance Segmentation, classe `blue_line`)
2. Esporta il modello in formato **ONNX** da Roboflow → Deploy → Export Model
3. Copia il file `.onnx` in questa cartella con il nome `trail_segmentation.onnx`
4. Aggiorna `model_path` in `sancho_params.yaml` se necessario

## Dipendenza Python

```bash
pip install onnxruntime
```

## Note

- I file `.onnx` NON sono committati nel repo (`.gitignore` li esclude) — vanno copiati manualmente sul rover
- Input atteso dal modello: `[1, 3, 432, 432]` float32, normalizzato 0-1
- Su Roboflow, impostare il resize a **432×432** nel dataset version (preprocessing step)
- Classe da estrarre: `blue_line`
