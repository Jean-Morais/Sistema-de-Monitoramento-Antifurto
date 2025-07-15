"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const express_1 = require("express");
const auth_1 = require("../middlewares/auth");
const validate_1 = require("../middlewares/validate");
const equipament_schema_1 = require("../schema/equipament.schema");
const equipament_controller_1 = require("./equipament.controller");
const equipamentRouter = (0, express_1.Router)();
// findAll
equipamentRouter.get('/', auth_1.authenticateToken, (0, validate_1.validateQuery)(equipament_schema_1.querySchema), equipament_controller_1.equipamentController.findAll);
// findById
equipamentRouter.get('/:id', auth_1.authenticateToken, (0, validate_1.validateParams)(equipament_schema_1.findByIdSchema), equipament_controller_1.equipamentController.findById);
// create new equipament
equipamentRouter.post('/', auth_1.authenticateToken, (0, validate_1.validateBody)(equipament_schema_1.createEquipmentSchema), equipament_controller_1.equipamentController.create);
// alert-out-of-range
equipamentRouter.post('/alert-out-of-range', auth_1.authenticateToken, (0, validate_1.validateBody)(equipament_schema_1.alertOutRangeSchema), equipament_controller_1.equipamentController.alertOutRange);
exports.default = equipamentRouter;
