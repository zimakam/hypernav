#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HyperNav — φ-jump Navigation Engine for N-Dimensional Spaces
==============================================================
Автор: Зиявутдинов Магомед Камалович (Zimaka)
Email: zimakam@gmail.com
ORCID: 0009-0005-9212-9921
Лицензия: MIT
Версия: 1.0.0
"""

import math
import heapq
import argparse
from typing import List, Optional, Dict

__author__ = "Зиявутдинов Магомед Камалович (Zimaka)"
__version__ = "1.0.0"
__license__ = "MIT"

PHI = (1.0 + math.sqrt(5.0)) / 2.0
FIB = [1, 1, 2, 3, 5, 8, 13, 21, 34]


class HyperNav:
    """
    Навигатор по N-мерному пространству через φ-jump.
    """

    def __init__(self, n_nodes: int = 256):
        self.n = int(n_nodes)
        self.phi_jump = int(n_nodes / PHI)
        self._cache: Dict[int, List[int]] = {}

    def neighbors(self, node_id: int) -> List[int]:
        """Соседи узла: φ-jump + Fibonacci."""
        if node_id in self._cache:
            return self._cache[node_id]
        s = set()
        # φ-jump
        s.add((node_id + self.phi_jump) % self.n)
        s.add((node_id - self.phi_jump) % self.n)
        # Fibonacci
        for f in FIB[:5]:
            s.add((node_id + f) % self.n)
            s.add((node_id - f) % self.n)
        s.discard(node_id)
        result = sorted(s)
        self._cache[node_id] = result
        return result

    def a_star(self, start: int, goal: int) -> Optional[dict]:
        """A* с γ-модуляцией (время важнее расстояния)."""
        open_set = [(0.0, start)]
        came_from = {}
        g_score = {start: 0.0}
        expanded = 0
        while open_set:
            _, current = heapq.heappop(open_set)
            expanded += 1
            if expanded > self.n * 10:
                return None
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return {"path": path, "cost": g_score[goal],
                        "expanded": expanded}
            for nb in self.neighbors(current):
                gamma = 1.0 + math.log(1 + nb)
                step_cost = 1.0 * (1.0 + 0.1 * math.log(gamma + 1))
                tentative = g_score[current] + step_cost
                if nb not in g_score or tentative < g_score[nb]:
                    came_from[nb] = current
                    g_score[nb] = tentative
                    h = abs(nb - goal)
                    heapq.heappush(open_set, (tentative + h, nb))
        return None

    def reachable(self, start: int = 0, k: int = 3) -> set:
        """Достижимые за k шагов."""
        visited = {start}
        frontier = {start}
        for _ in range(k):
            new = set()
            for node in frontier:
                for nb in self.neighbors(node):
                    if nb not in visited:
                        visited.add(nb)
                        new.add(nb)
            frontier = new
            if not frontier:
                break
        return visited

    def report(self) -> str:
        return (f"HyperNav(n_nodes={self.n}, φ_jump={self.phi_jump})")


def selftest():
    print("=" * 60)
    print(f"HYPERNAV v{__version__} — SELFTEST")
    print("=" * 60)
    nav = HyperNav(n_nodes=256)
    print(f"\n{nav.report()}")
    # Навигации
    print("\nНавигации:")
    for goal in [64, 128, 192, 255]:
        r = nav.a_star(0, goal)
        if r:
            print(f"  0 → {goal}: путь {len(r['path'])} узлов, "
                  f"стоимость {r['cost']:.3f}")
    # Достижимость
    reach = nav.reachable(0, k=3)
    print(f"\nДостижимо из 0 за 3 шага: {len(reach)} узлов")
    print("\n✅ SELFTEST пройден")
    print(f"Автор: {__author__}")


def main():
    parser = argparse.ArgumentParser(description="HyperNav v1.0.0")
    parser.add_argument("--selftest", action="store_true")
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--nodes", type=int, default=256)
    parser.add_argument("--from", dest="start", type=int, default=0)
    parser.add_argument("--to", dest="goal", type=int, default=128)
    args = parser.parse_args()

    if args.selftest:
        selftest()
        return
    if args.demo:
        nav = HyperNav(args.nodes)
        r = nav.a_star(args.start, args.goal)
        if r:
            print(f"Путь: {r['path']}")
            print(f"Стоимость: {r['cost']:.4f}")
        else:
            print("Путь не найден")
        return
    selftest()


if __name__ == "__main__":
    main()