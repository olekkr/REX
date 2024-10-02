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


# Local constants

Copy `empty_local_constants.py` and rename it to `local_constants.py`.

Any values you set in that will then be used instead of the ones in `constants.py`, for example if you `local_constants.py` looks like this:

```py
class LocalConstants:
    class Robot:
        pass

    class Sensor:
        pass

    class Obstacle:
        pass

    class PID:
        SCREEN_RESOLUTION = (640,480)

    class PyPlot:
        pass
```

then

```py
from constants import Constants

print(Constants.PID.FOCALLENGTH)  # prints the value from constants.py (1300)
print(Constants.PID.SCREEN_RESOLUTION)  # prints the value from local_constants.py (640,480)
```
