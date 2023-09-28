from enum import Enum


class Screens(Enum):
    MAIN = "main"
    CUSTOMIZATION = "customization"
    INSTRUCTIONS = "instructions"
    SONG_CHOICE = "song_choice"
    KING_OF_THE_BONGOS = "bongo game"
    TRAFFIC_LIGHT = "traffic light game"
    SIMON_SAYS = "simon says game"
    SCORE = "score"

class Songs(Enum):
    KOL_HAKAVOD = "kol_hakavod"
    PGISHA_BAMILUIM = "pgisha_bamiluim"
    KARNAVAL_BANACHAL = "karnaval_banachal"
    TRAFFIC_LIGHT_MUSIC = "traffic_light_music"
    SIMON_SAYS_MUSIC = "simon_says_music"

class Games(Enum):
    TRAFFIC_LIGHT = "Traffic Light"
    SIMON_SAYS = "Simon Says"
    KING_OF_THE_BONGOS = "King of the Bongos"

class SSPhase(Enum):
    RECALL_PHASE_CLICK_ENABLED = "Recall_click_enabled"
    RECALL_PHASE_CLICK_DISABLED = "Recall_click_disabled"
    TEACHING_PHASE = "Teaching"
