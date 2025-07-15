"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.validateQuery = exports.validateParams = exports.validateBody = void 0;
const validateBody = (schema) => {
    return (req, res, next) => {
        const result = schema.safeParse(req.body);
        if (!result.success) {
            return res.status(400).json({
                message: 'Dados inválidos',
                details: result.error.flatten(),
            });
        }
        // Anexa os dados validados para uso posterior
        req.body = result.data;
        next();
    };
};
exports.validateBody = validateBody;
const validateParams = (schema) => {
    return (req, res, next) => {
        const result = schema.safeParse(req.params);
        if (!result.success) {
            return res.status(400).json({
                message: 'Dados inválidos',
                details: result.error.flatten()
            });
        }
        // Anexa os dados validados para uso posterior
        req.params = result.data;
        next();
    };
};
exports.validateParams = validateParams;
const validateQuery = (schema) => {
    return (req, res, next) => {
        const result = schema.safeParse(req.query);
        if (!result.success) {
            return res.status(400).json({
                message: 'Dados inválidos',
                details: result.error.flatten()
            });
        }
        // substitui req.query com os dados validados (tipados e seguros)
        req.validatedQuery = result.data;
        next();
    };
};
exports.validateQuery = validateQuery;
