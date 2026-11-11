/*
 * VDP33 UI for Schwung
 *
 * Preset browsing through the shared sound-generator UI; parameter pages are
 * served by the DSP (get_param "ui_hierarchy"), which is the only place a
 * sound generator's hierarchy is read from.
 *
 * MIT License
 */

import { createSoundGeneratorUI } from '/data/UserData/schwung/shared/sound_generator_ui.mjs';

const ui = createSoundGeneratorUI({
    moduleName: 'VDP33',

    onPresetChange: () => {
        host_module_set_param('all_notes_off', '1');
    },

    showPolyphony: false,
    showOctave: true,
});

globalThis.init = ui.init;
globalThis.tick = ui.tick;
globalThis.onMidiMessageInternal = ui.onMidiMessageInternal;
globalThis.onMidiMessageExternal = ui.onMidiMessageExternal;
