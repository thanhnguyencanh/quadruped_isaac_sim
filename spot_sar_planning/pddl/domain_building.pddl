;; Spot SAR — TWO-FLOOR building domain (superset of domain_doors.pddl).
;;
;; Doors gate intra-floor traversal exactly as in the floor demo. A STAIR is the ONLY link between
;; the two (otherwise x-disjoint) floor wings, so STRIPS is FORCED to `use-stairs` to reach the
;; other floor — mirroring how `open-door` is forced before traversing a closed door. The stair is
;; actuated in the sim by an in-app teleport-assist (Spot's flat-terrain policy cannot climb steps).
;;
;; Passages between rooms are modelled as ALWAYS-OPEN doors (door-open in init) so `move` works
;; uniformly; only the two floor-1 doors start (door-closed), forcing an open-door.
;; door-between and stair-between are BOTH emitted in both orderings (a one-ordering emit makes
;; reverse traversal unsolvable — verified with the doors domain).
(define (domain spot-sar-building)
  (:requirements :strips :typing :negative-preconditions)

  (:types location door stair victim)

  (:predicates
    (at ?l - location)                                      ; robot is in room ?l
    (door-between ?d - door ?r1 - location ?r2 - location)  ; ?d joins ?r1 and ?r2 (emit BOTH orderings)
    (door-open ?d - door)                                   ; ?d is open (passages: always; doors: after open-door)
    (door-closed ?d - door)                                 ; ?d is still shut (init for the two floor-1 doors)
    (stair-between ?s - stair ?l1 - location ?l2 - location); ?s joins landings ?l1 and ?l2 (BOTH orderings)
    (explored ?l - location)                                ; ?l has been sensed
    (victim-at ?v - victim ?l - location)                   ; ?v is in room ?l
    (found ?v - victim)                                     ; ?v has been detected
    (reported ?v - victim))                                 ; ?v has been reported (the SAR goal)

  ;; intra-floor drive through an OPEN door / passage
  (:action move
    :parameters (?from - location ?to - location ?d - door)
    :precondition (and (at ?from) (door-between ?d ?from ?to) (door-open ?d))
    :effect (and (not (at ?from)) (at ?to)))

  ;; open a closed door while standing in one of the rooms it connects (physically actuated in sim)
  (:action open-door
    :parameters (?d - door ?r - location ?other - location)
    :precondition (and (at ?r) (door-between ?d ?r ?other) (door-closed ?d))
    :effect (and (door-open ?d) (not (door-closed ?d))))

  ;; change floors via the stairwell — the ONLY edge between the two wings (in-app teleport-assist).
  ;; Gated by standing AT the source landing, exactly as open-door is gated by being at the door.
  (:action use-stairs
    :parameters (?s - stair ?from - location ?to - location)
    :precondition (and (at ?from) (stair-between ?s ?from ?to))
    :effect (and (not (at ?from)) (at ?to)))

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
