use std::{
    cmp::Ordering,
    collections::{BinaryHeap, HashMap, HashSet},
    error::Error,
    fmt,
};

use screeps::{constants::Direction, game, local::RoomName};

#[derive(Debug, Clone, Copy)]
pub enum AnyResult {
    Fail,
}

impl fmt::Display for AnyResult {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl Error for AnyResult {}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
struct PortalRouterOpenSetEntry {
    /// the room in the open set
    room: RoomName,
    /// g_score represents the cost of the best known path to get to this node
    g_score: u32,
    /// f_score represents the estimated total cost of a path through this node,
    /// adding the best known cost to the node (the g_score) to the heuristic's estimate of the
    /// cost to get from the node to the goal
    f_score: u32,
    /// direction this entry was opened from
    open_dir: Option<Direction>,
}

impl Ord for PortalRouterOpenSetEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // we're using a max-heap but the behavior we want is min-heap, usual ordering is inverted
        other.f_score.cmp(&self.f_score)
    }
}

impl PartialOrd for PortalRouterOpenSetEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PortalRouterOpenSetEntry {
    pub fn new(
        room: RoomName,
        g_score: u32,
        open_dir: Option<Direction>,
        goals: &HashSet<RoomName>,
    ) -> Self {
        let heuristic_cost = get_heuristic_cost_to_closest_goal(room, goals);

        PortalRouterOpenSetEntry {
            room,
            g_score,
            f_score: g_score + heuristic_cost,
            open_dir,
        }
    }
}

/// Find cost as the lowest manhattan distance to any goal
fn get_heuristic_cost_to_closest_goal(room: RoomName, goals: &HashSet<RoomName>) -> u32 {
    let mut lowest_cost = u32::MAX;
    for goal in goals {
        let cost =
            room.x_coord().abs_diff(goal.x_coord()) + room.y_coord().abs_diff(goal.y_coord());
        if cost < lowest_cost {
            lowest_cost = cost;
        }
    }
    lowest_cost
}

/// navigate backwards across our map of where tiles came from to construct a path
fn resolve_completed_path(
    room_name: RoomName,
    visited: &HashMap<RoomName, Option<Direction>>,
) -> HashSet<RoomName> {
    let mut path = HashSet::new();
    path.insert(room_name);

    let mut cursor_room = room_name;

    while let Some(optional_search_direction) = visited.get(&cursor_room) {
        match optional_search_direction {
            Some(search_dir) => {
                if let Some(next_room) = cursor_room.checked_add((-*search_dir).into()) {
                    path.insert(next_room);
                    cursor_room = next_room;
                }
            }
            None => break,
        }
    }

    path
}

pub struct PortalRouterOps;

impl PortalRouterOps {
    pub fn find_route<F: Fn(&RoomName) -> u8>(
        origin: RoomName,
        goals: HashSet<RoomName>,
        cost_callback: F,
    ) -> Result<HashSet<RoomName>, AnyResult> {
        let mut open_set = BinaryHeap::new();
        // visited hashmap contains the direction we visited the room from, for backtracking once we find a path
        let mut visited = HashMap::new();

        open_set.push(PortalRouterOpenSetEntry::new(origin, 0, None, &goals));
        visited.insert(origin, None);

        while let Some(open_set_entry) = open_set.pop() {
            for direction in game::map::describe_exits(open_set_entry.room).keys() {
                // skip this direction quickly if it's toward the room that opened this entry
                if Some(-direction) == open_set_entry.open_dir {
                    continue;
                }
                if let Some(adj_room_name) = open_set_entry.room.checked_add(direction.into()) {
                    if visited.contains_key(&adj_room_name) {
                        continue;
                    }

                    // unvisited; check if goal first, then add open set entry if passable
                    visited.insert(adj_room_name, Some(direction));

                    if goals.contains(&adj_room_name) {
                        // we've found a goal; get the path back to it
                        let path = resolve_completed_path(adj_room_name, &visited);
                        return Ok(path);
                    }

                    let adj_traverse_cost = cost_callback(&adj_room_name);
                    if adj_traverse_cost < u8::MAX {
                        open_set.push(PortalRouterOpenSetEntry::new(
                            adj_room_name,
                            open_set_entry.g_score + adj_traverse_cost as u32,
                            Some(direction),
                            &goals,
                        ));
                    }
                }
            }
        }

        Err(AnyResult::Fail)
    }
}
