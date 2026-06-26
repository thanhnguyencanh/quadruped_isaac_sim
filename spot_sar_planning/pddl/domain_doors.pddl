;; Spot SAR — room-graph domain WITH openable doors (the "floor" demo, parallel to domain.pddl).
;;
;; Each ROOM is one location; inter-room links are DOORS. The robot may only `move` between two
;; rooms through an OPEN door, and the ONLY action whose effect opens a door is `open-door`. So
;; STRIPS forces the planner to `open-door` (a physical interaction in the sim) before it can
;; traverse into the next room. Verified with Fast Downward: removing `open-door` makes a
;; victim-in-another-room problem unsolvable.
;;
;; Doors carry two predicates (door-open / door-closed) so that `move` and `open-door` keep
;; POSITIVE preconditions (only open-door's *effect* deletes door-closed) — this keeps the parsed
;; problem.kind free of NEGATIVE_CONDITIONS and maximizes engine portability.
(define (domain spot-sar-doors)
  (:requirements :strips :typing :negative-preconditions)

  (:types location door victim)

  (:predicates
    (at ?l - location)                                      ; robot is in room ?l
    (door-between ?d - door ?r1 - location ?r2 - location)  ; ?d joins ?r1 and ?r2 (emit BOTH orderings)
    (door-open ?d - door)                                   ; ?d has been opened
    (door-closed ?d - door)                                 ; ?d is still shut (init for every door)
    (explored ?l - location)                                ; ?l has been sensed
    (victim-at ?v - victim ?l - location)                   ; ?v is in room ?l
    (found ?v - victim)                                     ; ?v has been detected
    (reported ?v - victim))                                 ; ?v has been reported (the SAR goal)

  ;; drive through an OPEN door from ?from to ?to
  (:action move
    :parameters (?from - location ?to - location ?d - door)
    :precondition (and (at ?from) (door-between ?d ?from ?to) (door-open ?d))
    :effect (and (not (at ?from)) (at ?to)))

  ;; open a closed door while standing in one of the rooms it connects (physically actuated in sim)
  (:action open-door
    :parameters (?d - door ?r - location ?other - location)
    :precondition (and (at ?r) (door-between ?d ?r ?other) (door-closed ?d))
    :effect (and (door-open ?d) (not (door-closed ?d))))

  ;; sense the current room (reveals victims here)
  (:action explore
    :parameters (?l - location)
    :precondition (at ?l)
    :effect (explored ?l))

  ;; confirm a victim once at and having explored its room
  (:action detect
    :parameters (?v - victim ?l - location)
    :precondition (and (at ?l) (explored ?l) (victim-at ?v ?l))
    :effect (found ?v))

  ;; report a found victim (the SAR goal)
  (:action report
    :parameters (?v - victim ?l - location)
    :precondition (and (at ?l) (found ?v) (victim-at ?v ?l))
    :effect (reported ?v)))
