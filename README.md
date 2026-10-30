# Will It Fit

A static website that checks whether a sofa (or any box-shaped object) can be moved through a tight space: a staircase that turns at the bottom, a corridor corner, or a doorway off a corridor. Enter the measurements, press **Find a way through**, and it searches for a route in 3D, including tipping the object on end and leaning it over things.

## Run it

```sh
python3 -m http.server 8000
# then open http://localhost:8000
```

## Files

- `index.html`: page, styles and UI (3D view, forms, playback)
- `sim.js`: geometry templates, collision checks and the route search (bidirectional RRT-Connect over position and orientation). Works in a browser or any JS engine.
- `vendor/`: three.js r128 and its OrbitControls (MIT licence), bundled so the site works without a CDN

Fonts load from Google Fonts. If that's unavailable, the page falls back to system fonts.

## How to read the result

- **Route found:** the object fits through these exact measurements, with the clearance you set.
- **No route found:** the search ran out of time. This strongly suggests it won't fit, but it isn't proof; very tight fits can need a longer search.
- **Search from this position:** place the object by hand with the sliders, then search from there. It checks your position to the destination first, then the hall to your position.

The sofa shape is modelled as two solid blocks (seat up to arm height plus a full-height backrest), so real cushions and removable legs give a little extra room.
