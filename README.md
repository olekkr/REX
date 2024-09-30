# REX

MVP GRUPPEN :D


# Run file using imports from parent folder

If we have the file structure

```
REX/
├─ folder/
│  ├─ sense.py
├─ constants.py
```

And we want to run `python refactor/sense.py` which imports from `constants.py`, the following lines can be added before the import

```py
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
```

This could also be done by adding the REX folder to the env variable PYTHONPATH, but since everyone running it would have to do it, this seems easier.
