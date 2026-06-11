# TODO

## Precision Landing
- [ ] **Figure out SEARCH behaviour**  
  When the drone loses sight of the marker (e.g. drifts too far, marker outside camera FOV) it enters SEARCH and hovers indefinitely. Need a strategy — options include a spiral search pattern, returning to a known offset above the marker's world position, or using GPS to fly back to the spawn point and re-acquire.
