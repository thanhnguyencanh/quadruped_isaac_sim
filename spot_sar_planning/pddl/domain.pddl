;; Spot SAR — STRIPS domain (Phase 5).
;; The PDDL planner (Fast Downward / ENHSP via unified-planning) selects the next high-level
;; action from the symbolic WorldModel produced by symbol grounding (spot_sar_planning/
;; world_model_node.py). Locations are symbolic cells; victims are grounded detections.
;;
;; Closed loop: the executive (Phase 6) regenerates the problem from /world_model each cycle
;; and replans as exploration reveals new locations and victims (partial observability).
(define (domain spot-sar)
  (:requirements :strips :typing :negative-preconditions)

  (:types location victim)

  (:predicates
    (at ?l - location)                       ; robot is at ?l
    (connected ?l1 - location ?l2 - location); ?l1 and ?l2 are adjacent (list both directions)
    (explored ?l - location)                 ; ?l has been explored
    (victim-at ?v - victim ?l - location)    ; ?v is located at ?l
    (found ?v - victim)                      ; ?v has been detected
    (reported ?v - victim))                  ; ?v has been reported

  ;; drive between adjacent locations
  (:action move
    :parameters (?from - location ?to - location)
    :precondition (and (at ?from) (connected ?from ?to))
    :effect (and (not (at ?from)) (at ?to)))

  ;; sense the current location (reveals victims here)
  (:action explore
    :parameters (?l - location)
    :precondition (at ?l)
    :effect (explored ?l))

  ;; confirm a victim once at and having explored its location
  (:action detect
    :parameters (?v - victim ?l - location)
    :precondition (and (at ?l) (explored ?l) (victim-at ?v ?l))
    :effect (found ?v))

  ;; report a found victim (the SAR goal)
  (:action report
    :parameters (?v - victim ?l - location)
    :precondition (and (at ?l) (found ?v) (victim-at ?v ?l))
    :effect (reported ?v)))
