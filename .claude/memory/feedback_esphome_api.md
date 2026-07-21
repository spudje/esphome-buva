---
name: feedback-esphome-api
description: ESPHome 2026.4 fan API breaking changes and how to handle them
metadata:
  type: feedback
---

In ESPHome 2026.4, the fan component API changed in breaking ways:

1. `FanTraits::set_supported_preset_modes()` is deprecated — call `this->set_supported_preset_modes()` on the Fan entity in `setup()` instead. Removed in 2026.11.0.

2. `Fan::preset_mode` (public `std::string`) became `Fan::preset_mode_` (private `const char*`). There is no public setter (`set_preset_mode()` does not exist on Fan). `get_preset_mode()` exists but is **not virtual**, so it cannot be overridden.

3. The only way to set `preset_mode_` from a custom component is via `FanCall::perform()`, which sets it internally before calling `control()`. Use this pattern for state updates (e.g. after polling):

```cpp
this->state_update_in_progress_ = true;
this->make_call().set_state(true).set_preset_mode("Low").perform();
this->state_update_in_progress_ = false;
this->publish_state();
```

Add a `state_update_in_progress_` guard in `control()` to skip the radio operation when doing a state-only update:

```cpp
if (this->state_update_in_progress_) {
    return;
}
```

**Why:** ESPHome privatised the field as part of moving to an optimistic state model where FanCall owns the state update lifecycle.

**How to apply:** Any custom ESPHome fan component targeting 2026.4+ needs these three changes.

4. A custom `get_traits()` override must call the protected `Fan::wire_preset_modes_(traits)` on
   the `FanTraits` instance before returning it. `set_supported_preset_modes()` (setup()) only
   stores the list on the `Fan` entity — `FanTraits` doesn't see it unless `get_traits()` wires it
   in on every call. Skipping this makes Home Assistant think the fan has zero preset modes
   (rejects `fan.set_preset_mode` client-side) and makes every internal
   `FanCall::set_preset_mode()` fail validation, logging "Preset mode 'X' not supported".

5. `FanCall::get_preset_mode()` returns `const char *` (nullable), not `std::string`/`optional`.
   Binding it directly to `const std::string &` constructs a temporary from a possibly-null
   pointer — UB/crash on any call without a preset set (e.g. a plain turn-on). Guard with
   `call.has_preset_mode()` before constructing a `std::string` from it.

**Confirmed 2026-07-21** against the actual esphome/esphome fan.h and fan.cpp source at tag
2026.4.5 while fixing exactly this pair of bugs in `components/zehnder_fan/zehnder_fan.cpp`.
