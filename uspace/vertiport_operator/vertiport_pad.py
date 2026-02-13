from sortedcontainers import SortedList

class Pad:
    def __init__(self, id, type, status, operator_id, location):
        self.id: str = id
        self.type: str = type
        self.status: str = status
        self.bookings: SortedList[tuple[float, float]] = SortedList()
        self.operator_id: str = operator_id
        self.location: tuple[float, float, float] = location

    def is_available(self, start_time: float, end_time: float, buffer: float):
        # Find the insertion point to maintain sorted order
        index = self.bookings.bisect_right((start_time, end_time))

        # Check for overlap with the previous booking
        if index > 0:
            _, prev_end = self.bookings[index - 1]

            if start_time < prev_end + buffer:
                return False
            
        # Check for overlap with the next booking
        if index < len(self.bookings):
            next_start, _ = self.bookings[index]

            if end_time > next_start:
                return False

        return True
    
    def book(
        self, 
        start_time: float, 
        end_time: float, 
        buffer: float, 
        availability_checked: bool=False
    ):
        # If availability not pre-checked, verify it now
        if not availability_checked and not self.is_available(start_time, end_time, buffer):
            return False

        # Insert the new booking while maintaining sorted order
        self.bookings.add((start_time, end_time))
        return True
    
    def get_bookings(self):
        return self.bookings
        
    def cancel_booking(self, start_time: float, end_time: float):
        self.bookings.discard((start_time, end_time))

    def update_booking(self, end_time: float):
        # Get (and remove) last booking entry
        last_book = self.bookings.pop(-1)

        # Update end time
        new_book = (last_book[0], end_time)

        # Re-insert updated booking
        self.bookings.add(new_book)
        