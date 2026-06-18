;; Example SAR problem: a 4-location corridor with one victim at the far end.
;; Optimal plan: move L0->L1->L2->L3, explore L3, detect v0, report v0.
;; (The executive generates problems like this automatically from /world_model.)
(define (problem sar-demo)
  (:domain spot-sar)
  (:objects
    L0 L1 L2 L3 - location
    v0 - victim)
  (:init
    (at L0)
    (connected L0 L1) (connected L1 L0)
    (connected L1 L2) (connected L2 L1)
    (connected L2 L3) (connected L3 L2)
    (victim-at v0 L3))
  (:goal (and (reported v0))))
