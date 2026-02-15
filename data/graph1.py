graph1 = {
    "Humera": ["Shire", "Gondar"],#fixed
    "Shire": ["Humera", "Debarke","Axum"],#fixed
    "Axum": ["Shire", "Adwa"],#fixed
    "Adwa": ["Axum", "Mekelle", "Adigrat"],#fized
    "Debarke": ["Humera","Shire", "Gondar"],#fized
    "Gondar": ["Debarke", "Metema", "Azezo","Humera"],#fixed
    "Metema": ["Azezo", "Gondar"],#fixed
    "Azezo": ["Gondar", "Bahir Dar","Metema"],#fixed
    "Bahir Dar": ["Azezo", "Debre Tabor", "Metekel", "Injibara", "Finote Selam"],#fixed
    "Mekelle":["Adwa","Adigrat","Alamata","Sekota"],#fixed
    "Assosa": ["Metekel", "Dembi Dollo"],#fixed
    "Dembi Dollo": ["Assosa", "Gimbi", "Gambela"],#fixed
    "Gambela": ["Dembi Dollo","Gore"],#fixed
    "Gore": ["Bedelle","Tepi"],#fixed
    "Gimbi": ["Dembi Dollo", "Nekemte"],#fixed
    "Nekemte": ["Gimbi", "Bedelle", "Ambo"],#fixed
    "Bedelle": ["Nekemte", "Gore","Jimma"],#fixed
    "Jimma": ["Bedelle", "Bonga", "Wolkite"],#fixed
    "Bonga": ["Jimma", "Tepi", "Dawro","Mezan Teferi"],#fixed
    "Tepi": ["Bonga","Mezan Teferi" "Gore"],#fixed
    "Gore": ["Tepi", "Gambela","Bedelle"],#fixed
    "Dawro": ["Bonga", "Wolaita Sodo"],#fixed
    "Wolaita Sodo": ["Dawro", "Hossana", "Arba Minch"],#fixed
    "Hossana": ["Wolaita Sodo", "Worabe","Shashemene"],#fixed
    "Worabe": ["Hossana", "Butajira","Wolkite"],#fixed
    "Butajira": ["Worabe", "Batu"],#fixed
    "Batu": ["Butajira", "Adama","Shashemene"],#fixed
    "Adama": ["Batu", "Addis Ababa", "Assella", "Matahara"],#fixed
    "Addis Ababa": ["Adama", "Ambo", "Debre Birhan"],#fixed
    "Ambo": ["Addis Ababa", "Wolkite", "Nekemte"],
    "Wolkite": ["Ambo", "Jimma","Worabe"],#fixed
    "Hawassa": [ "Dilla", "Shashemene"],#fixed
    "Dilla": ["Hawassa", "Bule Hora"],#fixed
    "Mezan Teferi":["Bonga","Basketo","Tepi"],
    "Basketo": ["Arba Minch", "Bench Maji","Mezan Teferi","Dawro"],#fixed
    "Bule Hora": ["Dilla", "Yabello"],#fxd
    "Yabello": ["Bule Hora", "Moyale", "Konso"],#fxd
    "Moyale": ["Yabello"],#fxd
    "Konso": ["Yabello", "Arba Minch"],#fixed
    "Bench Maji": ["Basketo"],#fxd

    "Debre Tabor": ["Bahir Dar", "Lalibela"],#fxd
    "Lalibela": ["Debre Tabor", "Woldia","Sekota"],#fixed
    "Woldia": ["Lalibela", "Dessie", "Alamata","Samara"],#fixed
    "Dessie": ["Woldia", "Kemise"],#fxd
    "Kemise": ["Dessie", "Debre Sina"],#fxd
    "Debre Sina": ["Kemise", "Debre Birhan","Debre Markos"],#fixed
    "Debre Birhan": ["Debre Sina", "Addis Ababa"],#fixed
    "Debre Markos": ["Debre Sina", "Finote Selam"],#fixed

    "Sekota": ["Mekelle", "Lalibela","Alamata"],#fixed
    "Alamata": ["Mekelle", "Woldia", "Sekota","Samara"],#fixed
    "Samara": ["Alamata", "Gabi Rasu", "Fanti Rasu","Woldia","Alamata"],#fixed
    "Fanti Rasu": ["Samara", "Kilbet Rasu"],#fxd
    "Kilbet Rasu": ["Fanti Rasu"],#fxd

    "Gabi Rasu": ["Samara", "Awash"],
    "Awash": ["Gabi Rasu", "Chiro", "Matahara"],
    "Chiro": ["Awash", "Dire Dawa"],
    "Dire Dawa": ["Chiro", "Harar"],
    "Harar": ["Dire Dawa", "Babile"],
    "Babile": ["Harar", "Jigjiga"],
    "Jigjiga": ["Babile", "Dega Habur"],
    "Dega Habur": ["Jigjiga", "Kebri Dehar","Goba"],#fxd
    "Goba": ["Dega Habur","Bale","Sof Oumer"],#fxd
    "Kebri Dehar": ["Dega Habur", "Gode", "Werder","Sof Oumer"],#fxd
    "Werder": ["Kebri Dehar"],
    "Gode": ["Kebri Dehar", "Dollo"],
    "Dollo": ["Gode"],

    "Assella": ["Adama", "Assasa"],
    "Assasa": ["Assella", "Dodolla"],
    "Dodolla": ["Assasa", "Bale","Shashemene"],#fxd
    "Bale": ["Dodolla", "Sof Oumer", "Liben","Goba"],
    "Liben": ["Bale"],
    "Sof Oumer": ["Bale", "Kebri Dehar","Goba"],#fxd

    "Shashemene": ["Hawassa", "Hossana","Batu","Dodolla"]
}
