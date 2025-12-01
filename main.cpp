#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <memory>

struct Particle {
    sf::Vector2f position;
    sf::Vector2f velocity;
    float radius;
    sf::Color color;
    
    Particle(float x, float y, float r) {
        position = sf::Vector2f(x, y);
        radius = r;
        
        // Kecepatan acak
        velocity.x = (rand() % 200 - 100) / 50.0f;
        velocity.y = (rand() % 200 - 100) / 50.0f;
        
        // Warna acak
        color = sf::Color(
            rand() % 256,
            rand() % 256,
            rand() % 256
        );
    }
    
    void update(float dt, float width, float height) {
        // Update posisi
        position += velocity * dt;
        
        // Collision dengan dinding
    
        const float RESTITUTION = 1.0f;
        
        if (position.x - radius < 0) {
            position.x = radius;
            velocity.x = -velocity.x * RESTITUTION;
        }
        if (position.x + radius > width) {
            position.x = width - radius;
            velocity.x = -velocity.x * RESTITUTION;
        }
        if (position.y - radius < 0) {
            position.y = radius;
            velocity.y = -velocity.y * RESTITUTION;
        }
        if (position.y + radius > height) {
            position.y = height - radius;
            velocity.y = -velocity.y * RESTITUTION;
        }
    }
    
    void draw(sf::RenderWindow& window) {
        sf::CircleShape shape(radius);
        shape.setPosition(position.x - radius, position.y - radius);
        shape.setFillColor(color);
        shape.setOutlineThickness(1);
        shape.setOutlineColor(sf::Color(255, 255, 255, 100));
        window.draw(shape);
    }
};

// batas Quadtree
struct Rectangle {
    float x, y, w, h;
    
    Rectangle(float _x, float _y, float _w, float _h) 
        : x(_x), y(_y), w(_w), h(_h) {}
    
    bool contains(const Particle& p) const {
        return (p.position.x >= x && p.position.x <= x + w &&
                p.position.y >= y && p.position.y <= y + h);
    }
    
    bool intersects(const Rectangle& range) const {
        return !(range.x > x + w || range.x + range.w < x ||
                 range.y > y + h || range.y + range.h < y);
    }
};

// algo Quadtree
class Quadtree {
private:
    static const int CAPACITY = 4;
    Rectangle boundary;
    std::vector<Particle*> particles;
    bool divided;
    
    std::unique_ptr<Quadtree> northwest;
    std::unique_ptr<Quadtree> northeast;
    std::unique_ptr<Quadtree> southwest;
    std::unique_ptr<Quadtree> southeast;
    
public:
    Quadtree(const Rectangle& rect) 
        : boundary(rect), divided(false) {
        particles.reserve(CAPACITY);
    }
    
    void subdivide() {
        float x = boundary.x;
        float y = boundary.y;
        float w = boundary.w / 2;
        float h = boundary.h / 2;
        
        northwest = std::make_unique<Quadtree>(Rectangle(x, y, w, h));
        northeast = std::make_unique<Quadtree>(Rectangle(x + w, y, w, h));
        southwest = std::make_unique<Quadtree>(Rectangle(x, y + h, w, h));
        southeast = std::make_unique<Quadtree>(Rectangle(x + w, y + h, w, h));
        
        divided = true;
    }
    
    bool insert(Particle* p) {
        if (!boundary.contains(*p)) {
            return false;
        }
        
        if (particles.size() < CAPACITY) {
            particles.push_back(p);
            return true;
        }
        
        if (!divided) {
            subdivide();
        }
        
        return (northwest->insert(p) || northeast->insert(p) ||
                southwest->insert(p) || southeast->insert(p));
    }
    
    void query(const Rectangle& range, std::vector<Particle*>& found) {
        if (!boundary.intersects(range)) {
            return;
        }
        
        for (auto p : particles) {
            if (range.contains(*p)) {
                found.push_back(p);
            }
        }
        
        if (divided) {
            northwest->query(range, found);
            northeast->query(range, found);
            southwest->query(range, found);
            southeast->query(range, found);
        }
    }
    
    void draw(sf::RenderWindow& window) {
        sf::RectangleShape rect(sf::Vector2f(boundary.w, boundary.h));
        rect.setPosition(boundary.x, boundary.y);
        rect.setFillColor(sf::Color::Transparent);
        rect.setOutlineColor(sf::Color(0, 255, 0, 50));
        rect.setOutlineThickness(1);
        window.draw(rect);
        
        if (divided) {
            northwest->draw(window);
            northeast->draw(window);
            southwest->draw(window);
            southeast->draw(window);
        }
    }
};

// cek collision partikel
bool checkCollision(Particle& p1, Particle& p2) {
    float dx = p2.position.x - p1.position.x;
    float dy = p2.position.y - p1.position.y;
    float distance = std::sqrt(dx * dx + dy * dy);
    
    return distance < (p1.radius + p2.radius);
}

// Fungsi untuk handle collision
void resolveCollision(Particle& p1, Particle& p2) {
    float dx = p2.position.x - p1.position.x;
    float dy = p2.position.y - p1.position.y;
    float distance = std::sqrt(dx * dx + dy * dy);
    
    if (distance == 0) return;
    
    float nx = dx / distance;
    float ny = dy / distance;
    
    float dvx = p2.velocity.x - p1.velocity.x;
    float dvy = p2.velocity.y - p1.velocity.y;
    
    float dvn = dvx * nx + dvy * ny;
    
    if (dvn > 0) return;    
    
    // Massa sesuai ukuran
    float m1 = p1.radius * p1.radius * p1.radius;
    float m2 = p2.radius * p2.radius * p2.radius;
    
    float impulse = 2 * dvn / (m1 + m2);
    
    p1.velocity.x += impulse * m2 * nx;
    p1.velocity.y += impulse * m2 * ny;
    p2.velocity.x -= impulse * m1 * nx;
    p2.velocity.y -= impulse * m1 * ny;
    
    // misahin partikel
    float overlap = (p1.radius + p2.radius) - distance;
    if (overlap > 0) {
        float separationX = nx * overlap * 0.5f;
        float separationY = ny * overlap * 0.5f;
        
        p1.position.x -= separationX;
        p1.position.y -= separationY;
        p2.position.x += separationX;
        p2.position.y += separationY;
    }
}

// algo Brute Force
int bruteForceCollision(std::vector<Particle>& particles, int& checks) {
    checks = 0;
    int collisions = 0;
    for (size_t i = 0; i < particles.size(); i++) {
        for (size_t j = i + 1; j < particles.size(); j++) {
            checks++;
            if (checkCollision(particles[i], particles[j])) {
                resolveCollision(particles[i], particles[j]);
                collisions++;
            }
        }
    }
    return collisions;
}

// detection Quadtree
int quadtreeCollision(std::vector<Particle>& particles, Quadtree& qtree, int& checks) {
    checks = 0;
    int collisions = 0;
    std::vector<std::pair<Particle*, Particle*>> checkedPairs;
    
    for (size_t i = 0; i < particles.size(); i++) {
        Particle& p = particles[i];
        
    
        float searchRadius = p.radius * 3;
        Rectangle searchArea(
            p.position.x - searchRadius,
            p.position.y - searchRadius,
            searchRadius * 2,
            searchRadius * 2
        );
        
        std::vector<Particle*> nearby;
        qtree.query(searchArea, nearby);
        
        // Cek collision hanya dengan partikel terdekat
        for (auto other : nearby) {
            if (&p < other) { // duplikasi
                checks++;
                if (checkCollision(p, *other)) {
                    resolveCollision(p, *other);
                    collisions++;
                }
            }
        }
    }
    return collisions;
}

int main() {
    srand(static_cast<unsigned>(time(0)));
    
    const int WIDTH = 1200;
    const int HEIGHT = 800;
    
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Simulasi Partikel - Brute Force vs Quadtree");
    window.setFramerateLimit(60);
    
        std::vector<Particle> particles;
    
    sf::Clock clock;
    bool useQuadtree = true;
    bool showQuadtree = false;
    int collisionChecks = 0;
    int totalCollisions = 0;
    int quadtreeCollisions = 0;
    int bruteforceCollisions = 0;
    
    // Font untuk teks
    sf::Font font;
    bool fontLoaded = font.loadFromFile("C:/project/DejaVuSans.ttf");
    
    sf::Text text;
    if (fontLoaded) {
        text.setFont(font);
    }
    text.setCharacterSize(16);
    text.setFillColor(sf::Color::White);
    text.setPosition(15, 15);
    text.setOutlineColor(sf::Color::Black);
    text.setOutlineThickness(2);
    
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
            
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Space) {
                    // Tambah partikel baru dengan kecepatan random
                    float radius = 10 + rand() % 30;
                    float x = WIDTH / 2.0f;
                    float y = 50.0f;
                    Particle newParticle(x, y, radius);
                    
                    // Buat arah random
                    float angle = (rand() % 360) * 3.14159f / 180.0f;
                    float speed = 200.0f + rand() % 300; // Speed 200-500
                    newParticle.velocity.x = std::cos(angle) * speed;
                    newParticle.velocity.y = std::sin(angle) * speed;
                    
                    particles.push_back(newParticle);
                }
                if (event.key.code == sf::Keyboard::R) {
                    // Reset
                    particles.clear();
                    totalCollisions = 0;
                    quadtreeCollisions = 0;
                    bruteforceCollisions = 0;
                }
                if (event.key.code == sf::Keyboard::Q) {
                    // Toggle algo
                    useQuadtree = !useQuadtree;
                }
                if (event.key.code == sf::Keyboard::V) {
                    // Toggle visual quadtree
                    showQuadtree = !showQuadtree;
                }
            }
        }
        
        float dt = clock.restart().asSeconds();
        
        // Update partikel
        for (auto& p : particles) {
            p.update(dt, WIDTH, HEIGHT);
        }
        
        // Collision Detection dengan algoritma yang dipilih
        int frameCollisions = 0;
        if (useQuadtree) {
            // Build Quadtree
            Quadtree qtree(Rectangle(0, 0, WIDTH, HEIGHT));
            for (auto& p : particles) {
                qtree.insert(&p);
            }
            
            // Quadtree detection
            frameCollisions = quadtreeCollision(particles, qtree, collisionChecks);
            quadtreeCollisions += frameCollisions;
            
            // Render
            window.clear(sf::Color(20, 20, 30));
            
            // Gambar Quadtree jika diaktifkan
            if (showQuadtree) {
                qtree.draw(window);
            }
        } else {
            // Brute Force detection
            frameCollisions = bruteForceCollision(particles, collisionChecks);
            bruteforceCollisions += frameCollisions;
            
            window.clear(sf::Color(20, 20, 30));
        }
        
        totalCollisions += frameCollisions;
        
        // Gambar partikel
        for (auto& p : particles) {
            p.draw(window);
        }
        
        // Info text
        std::string info = "=== SIMULASI PARTIKEL ===\n\n";
        info += "Algoritma: " + std::string(useQuadtree ? "QUADTREE" : "BRUTE FORCE") + "\n";
        info += "Partikel: " + std::to_string(particles.size()) + "\n";
        info += "FPS: " + std::to_string(static_cast<int>(1.0f / dt)) + "\n\n";
        
        info += "--- Per Frame ---\n";
        info += "Checks: " + std::to_string(collisionChecks) + "\n";
        info += "Collisions: " + std::to_string(frameCollisions) + "\n\n";
        
        info += "--- Total Collisions ---\n";
        info += "Quadtree: " + std::to_string(quadtreeCollisions) + "\n";
        info += "Brute Force: " + std::to_string(bruteforceCollisions) + "\n";
        info += "Total: " + std::to_string(totalCollisions) + "\n\n";
        
        info += "[SPACE] Tambah Partikel\n";
        info += "[Q] Toggle Algoritma\n";
        info += "[V] Visualisasi Quadtree\n";
        info += "[R] Reset (Hapus Semua)";
        
        text.setString(info);
        window.draw(text);
        
        window.display();
    }
    
    return 0;
}