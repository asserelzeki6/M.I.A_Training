
class Weapon:
    def __init__(self, name, energy_cost, damage, resources):
        # Constructor for the Weapon class
        self.name = name
        self.energy_cost = energy_cost
        self.damage = damage
        self.resources = resources

    def used(self):
        # Method to decrement the resources of the weapon when used
        self.resources -= 1

class Shield:
    def __init__(self, name, energy_cost, save_percentage, resources):
        # Constructor for the Shield class
        self.name = name
        self.energy_cost = energy_cost
        self.save_percentage = save_percentage
        self.resources = resources

    def used(self):
        # Method to decrement the resources of the shield when used
        self.resources -= 1

class Villain:
    def __init__(self, name, health=100, energy=500):
        # Constructor for the Villain class with default health and energy values
        self.name = name
        self.health = health
        self.energy = energy
        self.weapon = None
        self.shield = None
        self.weapons = []  # List to store available weapons
        self.shields = []  # List to store available shields

    def attack(self, person, kalman, magnet):
        # Method to perform an attack on another person (opponent)

        if self.energy >= self.weapon.energy_cost:
            # Check if enough energy is available for the attack
            self.energy -= self.weapon.energy_cost
            if not kalman:
                person.defend(self.weapon.damage, magnet)
            else:
                person.health -= self.weapon.damage
            self.weapon.used()  # Decrement the resources of the weapon
            return True  # Attack successful
        return False  # Attack failed due to lack of energy

    def defend(self, damage, magnet):
        # Method to defend against an attack and reduce health

        if self.shield:
            # Check if a shield is available for defense
            if magnet:
                # Apply additional damage reduction if magnet is active
                self.health -= (damage * (1 - (self.shield.save_percentage + 0.2)))
            else:
                self.health -= (damage * (1 - self.shield.save_percentage))
        else:
            if magnet:
                self.health -= (damage * (1 - 0.2))  # Apply magnet damage reduction only
            self.health -= damage  # Default damage reduction
        self.shield.used()  # Decrement the resources of the shield

# Create villains
gru = Villain("Gru")
vector = Villain("Vector")

# Assign weapons and shields to villains...
freeze_gun= Weapon("Freeze Gun",50,11,float('inf'))
electric_prod= Weapon("Electric Prod",88,18,5)
mega_magnet= Weapon("Mega Magnet",92,10,3)
kalman_missile= Weapon("Kalman MIssile",120,20,1)

energy_projected_barrier= Shield("Energy Projected BarrierGun", 20, 0.4,float('inf'))
selective_permeability= Shield("Selective Permeability", 50, 0.9, 2)

gru.weapons = [freeze_gun, electric_prod, mega_magnet, kalman_missile]
gru.shields = [energy_projected_barrier, selective_permeability]

laser_blasters= Weapon("Laser Blasters",40,8,float('inf'))
plasma_grenades= Weapon("Plasma Grenades",56,13,8)
sonic_resonance_cannon= Weapon("Sonic Resonance Cannon",100,22,3)

energy_net_trap= Shield("Energy Net Trap", 15, 0.32,float('inf'))
quantum_deflector= Shield("Quantum Deflector", 40, 0.8, 3)

vector.weapons = [laser_blasters, plasma_grenades, sonic_resonance_cannon]
vector.shields = [energy_net_trap, quantum_deflector]

# Define the game logic
class Game:
    def __init__(self, villain1, villain2):
        # Constructor for the Game class, initializing villains and other attributes
        self.villains = [villain1, villain2]
        self.rounds = 0
        self.magnet = False
        self.kalman = False

    def play_round(self):
        # Method to play a round of the game
        
        active = self.villains[0]  # Active villain in the round
        opponent = self.villains[1]  # Opponent villain
        
        # Display current villain information
        print(f"Round {self.rounds + 1}")
        print(f"{active.name}'s Health: {active.health}, Energy: {active.energy}")
        print(f"{opponent.name}'s Health: {opponent.health}, Energy: {opponent.energy}")
        
        # Allow active villain to choose weapons and shields for the round
        active.weapon = self.choose_weapon(active)
        if active.weapon == -1:
            return  # Return if weapon choice is invalid
        active.shield = self.choose_shield(active)
        if active.shield == -1:
            return  # Return if shield choice is invalid

        # Allow opponent villain to choose weapons and shields for the round
        opponent.weapon = self.choose_weapon(opponent)
        if opponent.weapon == -1:
            return  # Return if weapon choice is invalid
        opponent.shield = self.choose_shield(opponent)
        if opponent.shield == -1:
            return  # Return if shield choice is invalid

        # Check if the active villain uses Kalman Missile for attack
        self.kalman = active.weapon.name == "Kalman MIssile"

        # Perform attacks between active villain and opponent
        attack_case = active.attack(opponent, self.kalman, False)
        opponent.attack(active, False, self.magnet)

        # Reset magnet state
        if self.magnet:
            self.magnet = False

        # Check if Mega Magnet is used by the active villain
        if active.weapon.name == "Mega Magnet" and attack_case:
            self.magnet = True  # Set magnet state to True

        self.rounds += 1  # Increment the round count

        # Display the weapons and shields used by both villains in the round
        print(f"{active.name} used {active.weapon.name} and {active.shield.name}")
        print(f"{opponent.name} used {opponent.weapon.name} and {opponent.shield.name}")
        print()

    def choose_weapon(self, villain):
        # Method to allow a villain to choose a weapon for the round
        print(f"Available weapons for {villain.name}:")
        for i, weapon_ in enumerate(villain.weapons):
            print(f"{i + 1}. {weapon_.name} (Damage: {weapon_.damage}, Energy Cost: {weapon_.energy_cost})")

        choice = int(input("Choose a weapon (1, 2, 3, ...): "))
        
        if choice > len(villain.weapons)+1 or choice <= 0:
            print("Invalid weapon choice.")
            return -1  # Return -1 to indicate an invalid choice
        
        if villain.weapons[choice - 1].resources <= 0:
            print("You've run out of weapon resources.")
            return -1  # Return -1 if weapon resources are depleted
        
        villain.weapons[choice - 1].resources -= 1  # Decrement weapon resources
        return villain.weapons[choice - 1]  # Return the chosen weapon

    def choose_shield(self, villain):
        # Method to allow a villain to choose a shield for the round
        print(f"Available shields for {villain.name}:")
        for i, shield in enumerate(villain.shields):
            print(f"{i + 1}. {shield.name} (Save Percentage: {shield.save_percentage}, Energy Cost: {shield.energy_cost})")

        choice = int(input("Choose a shield (1, 2, 3, ...): "))
        
        if choice > len(villain.shields)+1 or choice <= 0:
            print("Invalid shield choice.")
            return -1  # Return -1 to indicate an invalid choice
        
        if villain.shields[choice - 1].resources <= 0:
            print("You've run out of shield resources.")
            return -1  # Return -1 if shield resources are depleted
        
        villain.shields[choice - 1].resources -= 1  # Decrement shield resources
        return villain.shields[choice - 1]  # Return the chosen shield


# Create the game and play rounds
game = Game(gru, vector)
while gru.health > 0 and vector.health > 0 and (gru.energy>50 or vector.energy>40):
    game.play_round()

if(gru.health>vector.health):
    print("Gru WINS")
else:
    print("Vector WINS")
print("Game Over")
