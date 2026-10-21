import pygame

from .settings import WHITE, MID, INK, ACTIVE

class NumericField:

    def __init__(self, key, value, parser=float, min_value=None, max_value=None):

        self.key = key
        self.value = value
        self.parser = parser

        self.min_value = min_value
        self.max_value = max_value

        self.text = self.format_value(value)

        self.rect = pygame.Rect(0, 0, 64, 30)

        self.active = False


    # -----------------------------
    # Format field value
    # -----------------------------

    def format_value(self, value):

        if isinstance(value, int):
            return str(value)

        return f"{value:g}"


    # -----------------------------
    # Set field position
    # -----------------------------

    def set_rect(self, rect):

        self.rect = pygame.Rect(rect)


    # -----------------------------
    # Handle field events
    # -----------------------------

    def handle_event(self, event):

        committed = False

        if (event.type == pygame.MOUSEBUTTONDOWN and event.button == 1):

            if self.rect.collidepoint(event.pos):

                self.active = True

            elif self.active:

                self.active = False

                committed = (self.commit())


        if (event.type == pygame.KEYDOWN and self.active):

            if event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):

                self.active = False

                committed = (self.commit())

            elif event.key == pygame.K_ESCAPE:

                self.text = (self.format_value(self.value))

                self.active = False

            elif event.key == pygame.K_BACKSPACE:

                self.text = (self.text[:-1])

            else:

                character = (event.unicode)

                if (character and character in "0123456789.-"):

                    self.text += (character)

        return committed


    # -----------------------------
    # Apply field value
    # -----------------------------

    def commit(self):

        try:

            value = self.parser(self.text)

            if self.min_value is not None:

                value = max(self.min_value, value)

            if self.max_value is not None:

                value = min(self.max_value, value)

            if self.parser is int:

                value = int(value)

            self.value = value

            self.text = (self.format_value(value))

            return True

        except (ValueError, TypeError):

            self.text = (self.format_value(self.value))

            return False


    # -----------------------------
    # Draw field
    # -----------------------------

    def draw(self, surface, font):

        mouse_position = (pygame.mouse.get_pos())

        hovered = (self.rect.collidepoint(mouse_position))

        fill = WHITE
        border = MID
        border_width = 1


        # -----------------------------
        # Hover field
        # -----------------------------

        if hovered:

            fill = (245, 245, 240)

            border = INK
            border_width = 2


        # -----------------------------
        # Selected field
        # -----------------------------

        if self.active:

            fill = (230, 240, 255)

            border = (0, 100, 255)

            border_width = 2


        pygame.draw.rect(surface, fill, self.rect)

        pygame.draw.rect(surface, border, self.rect, border_width)

        label = font.render(self.text, True, INK)

        surface.blit(label, label.get_rect(center=self.rect.center))

class Button:

    def __init__(self, label, action, tint=None, edge=None):

        self.label = label
        self.action = action

        self.tint = tint
        self.edge = edge

        self.rect = pygame.Rect(0, 0, 100, 34)


    # -----------------------------
    # Set button position
    # -----------------------------

    def set_rect(self, rect):

        self.rect = pygame.Rect(rect)


    # -----------------------------
    # Check click
    # -----------------------------

    def clicked(self, position):

        return self.rect.collidepoint(position)


    # -----------------------------
    # Draw button
    # -----------------------------

    def draw(self, surface, font, active=False):

        mouse_position = (pygame.mouse.get_pos())

        hovered = (self.rect.collidepoint(mouse_position))

        fill = (self.tint if self.tint is not None else WHITE)

        border = (self.edge if self.edge is not None else INK)

        border_width = 1
        text_colour = INK


        # -----------------------------
        # Hover state
        # -----------------------------

        if hovered and not active:

            if self.tint is not None:

                fill = tuple(max(0, value - 20) for value in self.tint)

            else:

                fill = (225, 225, 220)

            border_width = 3


        # -----------------------------
        # Selected state
        # -----------------------------

        if active:

            if self.edge is not None:

                fill = self.edge

            else:

                fill = ACTIVE

            border_width = 3
            text_colour = WHITE


        pygame.draw.rect(surface, fill, self.rect)

        pygame.draw.rect(surface, border, self.rect, border_width)

        text = font.render(self.label, True, text_colour)

        surface.blit(text, text.get_rect(center=self.rect.center))
