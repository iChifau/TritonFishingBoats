# CC35 Stepped Deep-V (Rhino 8 / Grasshopper)

This folder contains a parametric concept model for a 35 ft (10.67 m) stepped deep-V, Yellowfin-inspired center-console designed around:

- Units: meters
- LOA: 10.67 m (parametric)
- Beam: 3.20 m (parametric)
- Twin Yamaha F275 V6 3.3L (weight placeholders)
- Maldives: lagoon + offshore use; emphasis on dry ride (flare + spray rails)
- Helm electronics: (2) Simrad NSS evo3S displays (default: 2×12" with toggle to 2×16")
- Radar: Simrad Halo 24 (mount point placeholder)
- Transducer: Airmar M265LH (mounting keep-out zone)
- Fishboxes: parametric volumes with 75 mm insulation; sump points for macerator pumps and discharge guide curves

## Files
- `Triton_CC35_SteppedV_Rhino8.gh` – Grasshopper definition

## How to use
1. Open Rhino 8.
2. Set units to **meters**.
3. Open Grasshopper and load `Triton_CC35_SteppedV_Rhino8.gh`.
4. Bake layers/groups you want (Hull, Deck, Console, Hardtop, Fishboxes, Plumbing guides).

## Key parameters (in Grasshopper)
- LOA_m (default 10.67)
- Beam_m (default 3.20)
- Deadrise_transom_deg
- Deadrise_forward_deg
- Step_enable (bool)
- Step_position_ratio
- Step_height_m
- Flare_factor
- SprayRail_count (default 2)
- Console_width_m / Console_length_m
- Display_size_in (12 or 16) + Display_count (2)
- Fishbox_insulation_m (default 0.075)

## Notes / disclaimers
This is a *concept* geometry and layout tool. It is not a finished production design. Final structure (laminate schedule), stability, scantlings, systems design, and regulatory compliance must be completed by a qualified naval architect/boatbuilder.
