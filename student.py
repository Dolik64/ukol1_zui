from blockworld import BlockWorld
import heapq

from collections import defaultdict
import numpy as np
# Před importem blockworld "patchneme" alias np.int, aby odkazoval na np.int64.
np.int = np.int64

def canonical_state(state):
    """
    Převádí stav do kanonické, hashovatelné reprezentace.
    """
    return tuple(tuple(tower) for tower in state)

class BlockWorldHeuristic(BlockWorld):
    def __init__(self, pocet_bloku=5, state=None):
        super().__init__(pocet_bloku, state)

    def heuristic(self, cil):
        """
        Heuristika, která ignoruje pořadí věží a porovnává
        vždy věž ze startu s každou věží v cíli „odspodu nahoru“.
        Jakmile narazí na rozdíl, skončí a výslednou shodu už nezvyšuje.
        Následně vezme maximum shody pro každou věž.
        """

        # Pomocná funkce na zjištění, kolik bloků se shoduje odspodu.
        def spodni_shoda(vez_start, vez_cil):
            pocet_shod = 0
            while pocet_shod < len(vez_start) and pocet_shod < len(vez_cil):
                # Kontrolujeme i-tý blok odspodu (index od konce)
                if vez_start[-1 - pocet_shod] != vez_cil[-1 - pocet_shod]:
                    break
                pocet_shod += 1
            return pocet_shod

        # 1) Spočítáme celkový počet bloků v aktuálním stavu
        celkovy_pocet = 0
        index_veze = 0
        while index_veze < len(self.state):
            celkovy_pocet += len(self.state[index_veze])
            index_veze += 1

        # 2) Pro každou věž v current_state zjistíme nejlepší možnou
        #    shodu s libovolnou věží v goal.state
        soucet_max_shod = 0
        index_start = 0
        while index_start < len(self.state):
            aktualni_vez = self.state[index_start]
            nejlepsi_shoda_vez = 0

            index_cil = 0
            while index_cil < len(cil.state):
                cilova_vez = cil.state[index_cil]
                aktualni_shoda = spodni_shoda(aktualni_vez, cilova_vez)

                if aktualni_shoda > nejlepsi_shoda_vez:
                    nejlepsi_shoda_vez = aktualni_shoda

                index_cil += 1

            soucet_max_shod += nejlepsi_shoda_vez
            index_start += 1

        # 3) Heuristika = kolik bloků je „špatně“ = (celkový počet) - (suma shod)
        spatne_bloky = celkovy_pocet - soucet_max_shod
        return spatne_bloky


class AStar():
    def search(self, start, cil):
        """
        Prohledávání stavového prostoru metodou A*.
        Místo kanonického stavu (tuple) používáme prosté str(stav),
        protože to stačí a je čitelné.
        """
        # fronta s prioritami pro A*
        otevrena_pole = []
        
        # g_hodnota eviduje „cenu cesty“ od startu ke stavu
        g_hodnota = {}

        # Uzavřená množina (set) pro stavy, které jsme již prozkoumali
        uzavreno = set()

        # Přidáme startovní stav do fronty
        klic_startu = str(start.get_state())  # řetězec reprezentující stav
        g_hodnota[klic_startu] = 0
        zacatecni_odhad = start.heuristic(cil)
        heapq.heappush(otevrena_pole, (zacatecni_odhad, start, []))

        # Smyčka, dokud máme něco ve frontě
        while len(otevrena_pole) > 0:
            # Vezmeme stav s nejnižší hodnotou f (f = g + h)
            f_aktual, aktualni_stav, cesta_akci = heapq.heappop(otevrena_pole)
            klic_aktual = str(aktualni_stav.get_state())

            # Pokud už jsme tento stav zpracovali dřív, přeskočíme
            if klic_aktual in uzavreno:
                continue

            # Označíme stav jako uzavřený
            uzavreno.add(klic_aktual)

            # Ověříme, jestli jsme v cíli
            if aktualni_stav.get_state() == cil.get_state():
                return cesta_akci  # vracíme posloupnost akcí

            # Projdeme všechny sousedy (akce, sousední stavy)
            sousede = aktualni_stav.get_neighbors()
            index_souseda = 0
            while index_souseda < len(sousede):
                akce, dalsi_stav = sousede[index_souseda]
                index_souseda += 1

                klic_dalsi = str(dalsi_stav.get_state())
                nova_g = g_hodnota[klic_aktual] + 1

                # Pokud jsme do tohoto stavu ještě nešli
                # nebo jsme našli levnější cestu, aktualizujeme
                if (klic_dalsi not in g_hodnota) or (nova_g < g_hodnota[klic_dalsi]):
                    g_hodnota[klic_dalsi] = nova_g
                    h_odhad = dalsi_stav.heuristic(cil)
                    f_nova = nova_g + h_odhad
                    nova_cesta = cesta_akci + [akce]
                    heapq.heappush(otevrena_pole, (f_nova, dalsi_stav, nova_cesta))

        # Pokud jsme vyčerpali všechny možnosti a cíl jsme nenašli
        return None


    
if __name__ == '__main__':
	# Here you can test your algorithm. You can try different N values, e.g. 6, 7.
	N = 5

	start = BlockWorldHeuristic(N)
	goal = BlockWorldHeuristic(N)

	# start = BlockWorldHeuristic(N, state='[[2], [1], [4], [3, 5]]')
	# goal  = BlockWorldHeuristic(N, state='[[5, 3, 1], [2], [4]]')

	# print("Searching for a path:")
	# # Použití v hlavním kódu
	# print(convert_npint64_string(str(start)))
	# print(convert_npint64_string(str(goal)))

	#print(f"{start} -> {goal}")
	print()

	astar = AStar()
	path = astar.search(start, goal)

	if path is not None:
		print("Found a path:")
		print(path)

		print("\nHere's how it goes:")

		s = start.clone()
		print(s)

		for a in path:
			s.apply(a)
			print(s)

	else:
		print("No path exists.")

	print("Total expanded nodes:", BlockWorld.expanded)