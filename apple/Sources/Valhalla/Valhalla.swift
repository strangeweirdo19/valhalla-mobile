import ValhallaObjc
import ValhallaModels
import ValhallaConfigModels

public protocol ValhallaProviding {
    
    init(_ config: ValhallaConfig) throws
    
    init(configPath: String) throws

    func route(request: RouteRequest) throws -> RouteResponse
}

public final class Valhalla: ValhallaProviding {
    private let actor: ValhallaWrapper?
    private let configPath: String

    public convenience init(_ config: ValhallaConfig) throws {
        let configURL = try ValhallaFileManager.saveConfigTo(config)
        try self.init(configPath: configURL.relativePath)
    }

    public required init(configPath: String) throws {
        do {
            try ValhallaFileManager.injectTzdataIntoLibrary()
        } catch {
            // If you're circumventing this libraries injection, download tzdata.tar and put in your bundle. https://www.iana.org/time-zones
            fatalError("tzdata was not inject into Bundle.main. This can be avoided by including tzdata.tar in your main bundle.")
        }

        self.configPath = configPath
        do {
            self.actor = try ValhallaWrapper(configPath: configPath)
        } catch let error as NSError {
            throw ValhallaError.valhallaError(error.code, error.domain)
        } catch {
            throw ValhallaError.valhallaError(-1, error.localizedDescription)
        }
    }
    
    public func route(request: RouteRequest) throws -> RouteResponse {
        let requestData = try JSONEncoder().encode(request)
        guard let requestStr = String(data: requestData, encoding: .utf8) else {
            throw ValhallaError.encodingNotUtf8("requestStr")
        }
        
        let resultStr = route(rawRequest: requestStr)
        guard let resultData = resultStr.data(using: .utf8) else {
            throw ValhallaError.encodingNotUtf8("resultData")
        }
        
        if let error = try? JSONDecoder().decode(ValhallaErrorModel.self, from: resultData) {
            throw ValhallaError.valhallaError(error.code, error.message)
        }
        
        return try JSONDecoder().decode(RouteResponse.self, from: resultData)
    }

    public struct Structure: Codable, Equatable {
        public let type: String
        public let lat: Double
        public let lon: Double
        public let take: Bool
    }

    private struct TripEnvelope: Codable {
        struct Trip: Codable {
            let structures: [Structure]?
        }

        let trip: Trip?
        let structures: [Structure]?
    }

    /// Routes and returns bridge/tunnel structures from the first alternative separately.
    ///
    /// The native layer attaches a `structures` array to each trip in the JSON response.
    /// This returns the structures from the first (best) alternative.
    public func routeWithStructures(request: RouteRequest) throws -> (RouteResponse, [Structure]) {
        let requestData = try JSONEncoder().encode(request)
        guard let requestStr = String(data: requestData, encoding: .utf8) else {
            throw ValhallaError.encodingNotUtf8("requestStr")
        }

        let resultStr = route(rawRequest: requestStr)
        guard let resultData = resultStr.data(using: .utf8) else {
            throw ValhallaError.encodingNotUtf8("resultData")
        }

        if let error = try? JSONDecoder().decode(ValhallaErrorModel.self, from: resultData) {
            throw ValhallaError.valhallaError(error.code, error.message)
        }

        let route = try JSONDecoder().decode(RouteResponse.self, from: resultData)

        // Extract structures from trip (primary route) or the first alternative if available.
        var structures: [Structure] = []
        if let jsonObject = try? JSONSerialization.jsonObject(with: resultData) as? [String: Any] {
            if let rootStructuresData = try? JSONSerialization.data(withJSONObject: jsonObject),
               let envelope = try? JSONDecoder().decode(TripEnvelope.self, from: rootStructuresData) {
                if let tripStructures = envelope.trip?.structures, !tripStructures.isEmpty {
                    structures = tripStructures
                } else if let rootStructures = envelope.structures, !rootStructures.isEmpty {
                    structures = rootStructures
                }
            }

            if structures.isEmpty {
                let altKeys = ["alternates", "alternatives"]
                for key in altKeys {
                    if let alternatives = jsonObject[key] as? [[String: Any]],
                       !alternatives.isEmpty,
                       let firstAlt = alternatives.first,
                       let structuresData = try? JSONSerialization.data(withJSONObject: firstAlt),
                       let envelope = try? JSONDecoder().decode(TripEnvelope.self, from: structuresData) {
                        if let tripStructures = envelope.trip?.structures, !tripStructures.isEmpty {
                            structures = tripStructures
                        } else if let rootStructures = envelope.structures, !rootStructures.isEmpty {
                            structures = rootStructures
                        }
                        break
                    }
                }
            }
        }

        return (route, structures) // Always returns array (possibly empty)
    }

    public func route(rawRequest request: String) -> String {
        actor!.route(request)
    }
}
