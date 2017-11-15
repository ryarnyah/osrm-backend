# The route results with #original are what the result should be if the maneuver tag is removed
@routing @guidance @maneuver
Feature: Maneuver tag support

    Background:
        Given the profile "car"
        Given a grid size of 5 meters

    Scenario: simple override #1
        Given the node map
            """
            a--b---c----d---e
                   |
                   f
                   |
                   g
            """
        And the ways
            | nodes |
            | abc   |
            | cde   |
            | cfg   |

        And the relations
            | type     | way:from | node:via | way:to | maneuver | direction   |
            | maneuver | abc      | c        | cfg    | turn     | sharp_right |

        When I route I should get
            | waypoints | route       | turns                          |
            | a,g       | abc,cfg,cfg | depart,turn sharp right,arrive |


    Scenario: Use maneuver tag to announce a particular turn type
        Given the node map
            """
            f
            *
            *
             *
              *
                *
                  *
                    *
                     *
                      *
             t. ..     *                h
                  .. ....m**           *
                        /    *       *
                       /       * * *
                      /
                     /
                    |
                    |
                     \
                      \
                       o
            """

        And the ways
            | nodes | name      | oneway | highway      |
            | fm    | CA-120    | no     | secondary    |
            | mh    | CA-120    | no     | secondary    |
            | mt    | Priest Rd | no     | unclassified |
            | mo    |           | no     | service      |

        And the relations
            | type     | way:from | node:via | way:to | maneuver |
            | maneuver | mh       | m        | mt     | left     |

        When I route I should get
            | waypoints | route                        | turns                 |
            | h,t       | CA-120,Priest Rd,Priest Rd   | depart,left,arrive    |
  #original | h,t       | CA-120,Priest Rd,Priest Rd   | depart,turn straight,arrive  |

    Scenario: Use maneuver tag to announce lane guidance
        Given a grid size of 10 meters
        Given the node map
            """
               ad
              / \
             /   \
            /     \
            |     |
            |     |
            |     |
            b-----c------e
            |     |
            |     |
            |     |
            |     |
            r     w
            """

        And the ways
            | nodes | name      | oneway | highway   |
            | ab    | Marsh Rd  | yes    | secondary |
            | br    | Marsh Rd  | yes    | secondary |
            | cd    | Marsh Rd  | yes    | secondary |
            | cw    | Marsh Rd  | yes    | secondary |
            | bce   | service   | no     | service   |

        And the relations
            | type     | way:from | node:via | way:to | maneuver |
            | maneuver | ab       | b        | cd     | uturn    |

        When I route I should get
            | waypoints | route                         | turns                    |
            | a,d       | Marsh Rd,Marsh Rd,Marsh Rd    | depart,turn uturn,arrive |
  #original | a,d       | Marsh Rd,service,Marsh Rd,Marsh Rd | depart,turn left,turn left,arrive |

    Scenario: Use maneuver tag to suppress a turn
        Given the node map
            """
              c
              |
              |
          v---y----------z
              |
          n---p----------k
              |\
              | \
              b  t
            """

        And the ways
            | nodes | name    | oneway | highway       |
            | zy    | NY Ave  | yes    | primary       |
            | yv    | NY Ave  | yes    | primary       |
            | np    | NY Ave  | yes    | primary       |
            | pk    | NY Ave  | yes    | primary       |
            | cp    | 4th St  | no     | tertiary      |
            | yp    |         | no     | motorway_link |
            | pb    | 4th St  | no     | primary       |
            | pt    | 395     | no     | primary       |

        And the relations
            | type     | way:from | node:via | way:to | maneuver  |
            | maneuver | zy       | p        | pt     | suppress  |

        When I route I should get
            | waypoints | route           | turns                      |
            | z,t       | zy,yp,pt        | depart, left, arrive       |
  #original | z,t       | NY Ave,,395,395 | depart,on ramp left,fork slight left,arrive |

